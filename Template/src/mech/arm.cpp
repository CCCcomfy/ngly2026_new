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

double targetDeg = 0.0;
double offsetDeg = 0.0;   // 编码器零位偏移，tare() 改它
bool active = false;
bool holding = false;     // 到位后继续托住
bool atTarget = true;
double lastOutMv = 0.0;
bool manualMode = false;
double manualMv = 0.0;
std::uint32_t startMs = 0;
std::uint32_t settleMs = 0;

// 是否归零过：没归零时 getAngle() 的 0 不代表真实角度，软限位不能用它判断。
bool zeroed = false;

constexpr double kPi = 3.14159265358979323846;

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline double rawDeg() {
    return motorA.get_position() / ARM_MOTOR_DEG_PER_ARM_DEG;
}

inline void writeBoth(double mv) {
    const int32_t v = static_cast<int32_t>(mv);
    motorA.move_voltage(v);
    motorB.move_voltage(v);
}

// 臂越接近水平重力矩越大，竖直时几乎为零。用固定值会水平抖、竖直冲。
inline double gravityFeed(double deg) {
    const double c = std::cos(deg * kPi / 180.0);
    return ARM_KG_MV * (c > 0.0 ? c : 0.0);
}

inline double limited(double mv) {
    const double cap = arm::getTemperature() > TEMP_LIMIT_C ? ARM_MAX_MV * 0.5 : ARM_MAX_MV;
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
    manualMv = 0.0;
    zeroed = false;   // 刚上电，角度不可信
}

void arm::update() {
    // 手动直通模式：绕开 PID，但仍然保留限位、过温和 slew
    // 手柄推着摇臂走的时候走这条分支；松手立刻切回 PID 托住。
    if (manualMode) {
        // 软限位只在「归零过 + 编码器有数」时才生效。
        // 没归零时 0 度不代表真实角度；端口没插电机时 getAngle() 会算出 inf/nan。
        // 否则按键一按就被判越界清零，看起来就是「按键没反应」。
        const double deg = getAngle();
        if (zeroed && std::isfinite(deg)) {
            if ((deg <= ARM_MIN_DEG && manualMv < 0) || (deg >= ARM_MAX_DEG && manualMv > 0)) {
                manualMv = 0;   // 软限位：快撞到头就断电，否则会顶弯结构
            }
        }
        writeBoth(limited(manualMv));
        return;
    }

    if (!active && !holding) {
        return;
    }

    const double deg = getAngle();
    const double err = targetDeg - deg;

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

        // 超时不能刹车，一松臂就砸下来，改成原地托住
        if (active && pros::millis() - startMs > static_cast<std::uint32_t>(ACTION_TIMEOUT_MS)) {
            active = false;
            holding = true;
            atTarget = false;
        }
    }

    writeBoth(limited(ARM_KP_MV_PER_DEG * err + gravityFeed(deg)));
}

// 非阻塞异步版本
void arm::toAngleAsync(double deg) {
    if (deg < ARM_MIN_DEG || deg > ARM_MAX_DEG) {
        return;
    }
    targetDeg = deg;
    active = true;
    holding = false;
    atTarget = false;
    settleMs = 0;
    startMs = pros::millis();
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
        manualMode = false;
        hold();
        return;
    }

    manualMode = true;
    manualMv = static_cast<double>(mv);
    active = false;
    holding = false;
    atTarget = false;
}

void arm::stop() {
    // 同样不能断电，把目标设成当前角度继续托住
    manualMode = false;
    targetDeg = getAngle();
    active = false;
    holding = true;
    atTarget = false;
}

void arm::hold() {
    manualMode = false;
    targetDeg = getAngle();
    active = false;
    holding = true;
}

bool arm::isBusy() { return active; }

bool arm::isAtTarget() { return atTarget; }

bool arm::sensorOk() { return std::isfinite(getAngle()); }

bool arm::isZeroed() { return zeroed; }

double arm::getAngle() { return rawDeg() - offsetDeg; }

double arm::getTemperature() {
    return std::max(motorA.get_temperature(), motorB.get_temperature());
}

void arm::tare() {
    offsetDeg = rawDeg();
    targetDeg = 0.0;
    zeroed = true;   // 从这里开始，角度才是真实角度
}

// 慢速往下压，撞到机械限位后速度归零，把那个位置定义成 ARM_MIN_DEG。
bool arm::zeroByStall(int timeoutMs) {
    manualMode = false;   // 不清掉，归零结束后 update 会拿旧的手柄电压接着压
    manualMv = 0.0;
    active = false;
    holding = false;
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
                return true;
            }
        } else {
            stallMs = 0;
        }
        pros::delay(20);
    }
    writeBoth(0);
    holding = true;
    atTarget = false;
    return false;
}
