// 摇臂（旋转轴）。
// 和滑轨的区别：摇臂有重力且随角度变化，所以前馈要乘 cos；
// 到位后不能退出，得继续托住——断电臂会直接砸下来。

#include "mech/mech.hpp"

#include <algorithm>
#include <cmath>

#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace {

// PROS 3.8 的构造是 (port, gearset, reversed, encoder_units)，没有 PROS 4 的枚举。
pros::Motor motorA(PORT_ARM_A, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motorB(PORT_ARM_B, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

struct PidState {
    double integral = 0.0;
    double prevMeas = 0.0;
    bool hasPrev = false;

    void reset() {
        integral = 0.0;
        hasPrev = false;
    }
};

double targetDeg = 0.0;
double offsetDeg = 0.0;   // 编码器零位偏移，tare() 改它
bool active = false;
bool holding = false;     // 到位后继续托住
bool atTarget = true;
double lastOutMv = 0.0;
bool manualMode = false;
bool stopping = false;          // 松手后由速度环刹车，速度归零再切位置保持
double manualMv = 0.0;
double manualVelTarget = 0.0;   // 手动满杆 120 deg/s，按杆量线性缩放
PidState posPid;                // 位置环
PidState velPid;                // 手动速度环
double lastVel = 0.0;           // 由角度差分得到的角速度，符号和 getAngle 一致
double lastDeg = 0.0;
std::uint32_t lastDegMs = 0;
bool hasLastDeg = false;
std::uint32_t startMs = 0;
std::uint32_t settleMs = 0;

// 是否归零过：没归零时 getAngle() 的 0 不代表真实角度，软限位不能用它判断。
bool zeroed = false;

constexpr double kPi = 3.14159265358979323846;

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 通用 PID：微分作用在测量值上（derivative-on-measurement），避免换目标时
// 目标阶跃把 D 项打满；积分带条件抗饱和，输出顶到限幅时不再继续积分。
inline double pidStep(PidState& s, double err, double meas, double dt,
                      double kp, double ki, double kd, double ff,
                      double lo, double hi) {
    if (!s.hasPrev) {
        s.prevMeas = meas;
        s.hasPrev = true;
    }
    const double deriv = (meas - s.prevMeas) / dt;
    double integ = s.integral + err * dt;
    double out = ff + kp * err + ki * integ - kd * deriv;

    if (out > hi) {
        out = hi;
        if (err > 0.0) {
            integ = s.integral;   // 正误差还在把输出往上顶，别再积
        }
    } else if (out < lo) {
        out = lo;
        if (err < 0.0) {
            integ = s.integral;
        }
    }

    s.integral = clampd(integ, -ARM_I_LIMIT_DEG_SEC, ARM_I_LIMIT_DEG_SEC);
    s.prevMeas = meas;
    return out;
}

constexpr double kDt = UPDATE_PERIOD_MS / 1000.0;

inline double rawDeg() {
    return motorA.get_position() / ARM_MOTOR_DEG_PER_ARM_DEG;
}

double lastCmdMv = 0.0;   // 最近一次真正写给电机的电压，给屏幕 CMD 读数用

inline void writeBoth(double mv) {
    const int32_t v = static_cast<int32_t>(mv);
    lastCmdMv = mv;
    motorA.move_voltage(v);
    motorB.move_voltage(v);
}

// 臂越接近水平重力矩越大，竖直时几乎为零。用固定值会水平抖、竖直冲。
inline double gravityFeed(double deg) {
    const double c = std::cos(deg * kPi / 180.0);
    return ARM_KG_MV * (c > 0.0 ? c : 0.0);
}

// 角速度不再读电机 RPM：A/B 有一台 reversed 时符号可能相反，平均会抵消成 0，
// 表现就是负角度松手刹不住。直接用 getAngle() 差分，符号和位置环完全一致。
inline double updateVelocity(double deg) {
    const std::uint32_t now = pros::millis();
    if (!hasLastDeg || now - lastDegMs > 100) {
        lastDeg = deg;
        lastDegMs = now;
        hasLastDeg = true;
        lastVel = 0.0;
        return 0.0;
    }
    const double dt = (now - lastDegMs) / 1000.0;
    double v = 0.0;
    if (dt > 1e-4) {
        v = (deg - lastDeg) / dt;
    }
    if (!std::isfinite(v)) {
        v = 0.0;
    }
    v = clampd(v, -1000.0, 1000.0);
    lastVel = 0.7 * lastVel + 0.3 * v;
    lastDeg = deg;
    lastDegMs = now;
    return lastVel;
}

inline double limited(double mv) {
    // 编码器没数时 err 可能算出 nan/inf；转成 int32 是未定义行为，会一脚踹满
    if (!std::isfinite(mv)) {
        mv = 0.0;
    }
    const double temp = arm::getTemperature();
    const double cap =
        (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? ARM_MAX_MV * 0.5 : ARM_MAX_MV;
    double out = clampd(mv, -cap, cap);
    const double step = ARM_SLEW_MV_PER_SEC * UPDATE_PERIOD_MS / 1000.0;
    out = clampd(out, lastOutMv - step, lastOutMv + step);
    lastOutMv = out;
    return out;
}

}  // namespace

void arm::init() {
    motorB.set_reversed(ARM_B_REVERSED);
    motorA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorA.tare_position();
    motorB.tare_position();

    offsetDeg = 0.0;
    active = false;
    holding = false;
    atTarget = true;
    lastOutMv = 0.0;
    manualMode = false;
    stopping = false;
    manualMv = 0.0;
    manualVelTarget = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    zeroed = false;   // 刚上电，角度不可信
}

void arm::update() {
    // 手动模式：手柄给的是“速度目标”，不是直接电压。
    // 速度 PI + 重力前馈，既保留托举力，又不会在负角度一按就满压硬顶/抖动。
    if (manualMode || stopping) {
        const double deg = getAngle();
        if (!std::isfinite(deg)) {
            manualMode = false;
            stopping = false;
            manualVelTarget = 0.0;
            lastOutMv = 0.0;
            writeBoth(0);
            return;
        }

        const double assist = gravityFeed(deg);
        const double vel = updateVelocity(deg);
        double targetVel = manualMode ? manualVelTarget : 0.0;

        // 软限位只在「归零过」时才可信；到限位就只保留托举，不再往外顶。
        if (zeroed) {
            if (deg <= ARM_MIN_DEG && targetVel < 0.0) {
                targetVel = 0.0;
            } else if (deg >= ARM_MAX_DEG && targetVel > 0.0) {
                targetVel = 0.0;
            }
        }

        const double kv = ARM_MANUAL_MV / ARM_MANUAL_MAX_DPS;
        const double ff = assist + kv * targetVel;
        const double temp = getTemperature();
        const double cap =
            (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? ARM_MAX_MV * 0.5 : ARM_MAX_MV;
        // 刹车阶段不给积分：那一段 err 一直是负的，积分会把刹车越刹越狠，刹成反向冲。
        const double ki = manualMode ? ARM_MANUAL_KI_MV_PER_DPS_SEC : 0.0;
        double out = pidStep(velPid, targetVel - vel, vel, kDt,
                             ARM_MANUAL_KP_MV_PER_DPS, ki, 0.0,
                             ff, -cap, cap);

        // 限位处只允许托住，不允许继续往外压。
        if (zeroed) {
            if (deg <= ARM_MIN_DEG && out < assist) {
                out = assist;
            } else if (deg >= ARM_MAX_DEG && out > assist) {
                out = assist;
            }
        }
        writeBoth(limited(out));

        // 速度压到接近 0 才交给位置环：这一步是「松手不砸」的关键。
        // 直接切 hold 的话，手动满压的 lastOutMv 和保持电压差着 6000 mV，
        // 斜率限幅要爬好几拍，臂早就在这个空档里掉下去了。
        if (stopping && std::fabs(vel) < ARM_STOP_DPS) {
            stopping = false;
            targetDeg = deg;
            active = false;
            holding = true;
            atTarget = true;
            posPid.reset();
            velPid.reset();
            hasLastDeg = false;
            lastVel = 0.0;
        }
        return;
    }

    if (!active && !holding) {
        return;
    }

    const double deg = getAngle();
    // 编码器掉线时不能拿 NaN 去算 err，否则会一路踹到满压。
    if (!std::isfinite(deg)) {
        manualMode = false;
        active = false;
        holding = false;
        atTarget = false;
        lastOutMv = 0.0;
        writeBoth(0);
        return;
    }
    const double err = targetDeg - deg;
    updateVelocity(deg);   // 给 hold 的第一拍刹车保留一个和角度同符号的速度

    if (active) {
        // 误差够小还要稳 80ms，否则在临界点反复启停
        if (std::fabs(err) < ARM_TOL_DEG) {
            settleMs += UPDATE_PERIOD_MS;
            if (settleMs >= 80) {
                active = false;
                holding = true;   // 从「转过去」切成「托住」
                atTarget = true;
            }
        } else {
            settleMs = 0;
        }

        // 超时不能刹车，一松臂就砸下来，改成原地托住。
        // 目标必须改成当前角度，否则 holding 还在追一个够不到的目标。
        if (active && pros::millis() - startMs > static_cast<std::uint32_t>(ACTION_TIMEOUT_MS)) {
            targetDeg = deg;
            active = false;
            holding = true;
            atTarget = false;
            posPid.reset();
            return;
        }
    }

    // 位置 PID：P 负责刚度，I 补静摩擦/电池差异，D 压超调；重力前馈单独加。
    const double temp = getTemperature();
    const double cap =
        (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? ARM_MAX_MV * 0.5 : ARM_MAX_MV;
    const double out = pidStep(posPid, err, deg, kDt,
                               ARM_KP_MV_PER_DEG, ARM_KI_MV_PER_DEG_SEC,
                               ARM_KD_MV_PER_DEG_PER_SEC, gravityFeed(deg),
                               -cap, cap);
    writeBoth(limited(out));
}

// 非阻塞异步版本
void arm::toAngleAsync(double deg) {
    if (!std::isfinite(deg) || deg < ARM_MIN_DEG || deg > ARM_MAX_DEG) {
        return;
    }
    targetDeg = deg;
    active = true;
    holding = false;
    atTarget = false;
    settleMs = 0;
    startMs = pros::millis();
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
}

// 阻塞同步版本
void arm::toAngle(double deg) {
    toAngleAsync(deg);
    while (active) {
        pros::delay(5);
    }
}

void arm::nudge(double deltaDeg) {
    toAngleAsync(targetDeg + deltaDeg);
}

void arm::manual(int mv) {
    if (mv == 0) {
        if (!manualMode) {
            return;      // 本来就没在手动，什么都不做
        }
        // 不直接切 hold：先交给速度环把速度拉到 0，再由 update() 交棒给位置环。
        manualMode = false;
        stopping = true;
        manualVelTarget = 0.0;
        return;
    }

    const bool wasManual = manualMode;
    const bool reversed = (manualMv * static_cast<double>(mv)) < 0.0;
    manualMode = true;
    stopping = false;
    manualMv = static_cast<double>(mv);
    manualVelTarget = (static_cast<double>(mv) / ARM_MANUAL_MV) * ARM_MANUAL_MAX_DPS;
    if (!wasManual || reversed) {
        velPid.reset();
        posPid.reset();
        hasLastDeg = false;
        lastVel = 0.0;
    }
    active = false;
    holding = false;
    atTarget = false;
}

void arm::stop() {
    // 同样不能断电，把目标设成当前角度继续托住
    manualMode = false;
    stopping = false;
    const double deg = getAngle();
    if (!std::isfinite(deg)) {
        active = false;
        holding = false;
        lastOutMv = 0.0;
        writeBoth(0);
        return;
    }
    targetDeg = deg;
    active = false;
    holding = true;
    atTarget = false;
    manualVelTarget = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
}

void arm::hold() {
    manualMode = false;
    stopping = false;
    const double deg = getAngle();
    if (!std::isfinite(deg)) {
        active = false;
        holding = false;
        lastOutMv = 0.0;
        writeBoth(0);
        return;
    }
    targetDeg = deg;
    // 注意：这里不要动 lastOutMv。
    // 以前把「反向刹车」预装进斜率累加器，一松手就是 -8000 mV 的硬反转，
    // 斜率限幅要 0.5 秒才爬回保持电压 —— 表现就是「停住瞬间急剧下落 + 来回晃」。
    // 现在松手走 stopping（速度环刹到 0），电压全程连续。
    active = false;
    holding = true;
    manualVelTarget = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
}

bool arm::isBusy() { return active; }

bool arm::isAtTarget() { return atTarget; }

bool arm::sensorOk() { return std::isfinite(getAngle()); }

bool arm::isZeroed() { return zeroed; }

double arm::lastCommandMv() { return lastCmdMv; }

double arm::getAngle() { return rawDeg() - offsetDeg; }

double arm::getTemperature() {
    return std::max(motorA.get_temperature(), motorB.get_temperature());
}

void arm::tare() {
    // 只有真的在水平位才能调这个函数；在底部 tare 会把水平定义成 -ARM_MIN_DEG。
    const double raw = rawDeg();
    if (!std::isfinite(raw)) {
        return;
    }
    offsetDeg = raw;
    targetDeg = 0.0;
    zeroed = true;   // 从这里开始，角度才是真实角度
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
}

// 慢速往下压，撞到机械限位后速度归零，把那个位置定义成 ARM_MIN_DEG。
bool arm::zeroByStall(int timeoutMs) {
    manualMode = false;   // 不清掉，归零结束后 update 会拿旧的手柄电压接着压
    stopping = false;
    manualMv = 0.0;
    manualVelTarget = 0.0;
    active = false;
    holding = false;
    lastOutMv = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t stallMs = 0;

    while (pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
        writeBoth(-3000);
        if (std::fabs(motorA.get_actual_velocity()) < 3.0) {
            stallMs += 20;
            if (stallMs >= 300) {
                writeBoth(0);
                pros::delay(150);
                offsetDeg = rawDeg() - ARM_MIN_DEG;
                targetDeg = ARM_MIN_DEG;
                holding = true;
                atTarget = true;
                zeroed = true;   // 压到底了，ARM_MIN_DEG 就是真实下限
                lastOutMv = 0.0;
                return true;
            }
        } else {
            stallMs = 0;
        }
        pros::delay(20);
    }
    writeBoth(0);
    // 失败时不能留着旧目标，否则下一次 update 会继续往那个位置顶。
    const double deg = getAngle();
    if (std::isfinite(deg)) {
        targetDeg = deg;
        holding = true;
    } else {
        holding = false;
    }
    lastOutMv = 0.0;
    atTarget = false;
    return false;
}
