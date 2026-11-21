// 滑轨（竖直升降）。
// 双电机带同一根轴，只读 A 的编码器算位置，同一个电压写给两个电机。
// 各自跑 PID 会互相拉扯，位置永远收敛不了。

#include "mech/mech.hpp"

#include <algorithm>
#include <cmath>

#include "pros/motors.hpp"
#include "pros/rtos.hpp"

namespace {

// PROS 3.8 的构造是 (port, gearset, reversed, encoder_units)，
// 没有 PROS 4 的 MotorGears / MotorUnits 枚举，所以直接写 18:1 绿齿轮箱。
pros::Motor motorA(PORT_SLIDE_A, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motorB(PORT_SLIDE_B, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

double targetIn = 0.0;
bool active = false;
bool atTarget = true;
double lastOutMv = 0.0;
bool manualMode = false;
double manualMv = 0.0;
std::uint32_t startMs = 0;
std::uint32_t settleMs = 0;

// 是否归零过。没归零时 getInches() 的 0 只是「上电时停在哪」，
// 拿它当行程零点本身就是错的，软限位也就不能用它来判断。
bool zeroed = false;

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline void writeBoth(double mv) {
    const int32_t v = static_cast<int32_t>(mv);
    motorA.move_voltage(v);
    motorB.move_voltage(v);
}

inline void brakeBoth() {
    writeBoth(0);
    motorA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lastOutMv = 0.0;   // 不清掉，下次启动会带着上次的电压冲一下
}

// 过温降功率 + 限制电压爬升。手动和闭环两个分支都得过这一关。
inline double limited(double mv) {
    const double cap = slide::getTemperature() > TEMP_LIMIT_C ? SLIDE_MAX_MV * 0.5 : SLIDE_MAX_MV;
    double out = clampd(mv, -cap, cap);
    const double step = SLIDE_SLEW_MV_PER_SEC * UPDATE_PERIOD_MS / 1000.0;
    out = clampd(out, lastOutMv - step, lastOutMv + step);
    lastOutMv = out;
    return out;
}

}  // namespace

void slide::init() {
    // 整轴方向 + B 相对 A 的极性。写成异或的「相对关系」：
    // 翻整轴（SLIDE_DIR_REVERSED）时，B 与 A 的相对极性保持不变。
    motorA.set_reversed(SLIDE_DIR_REVERSED);
    motorB.set_reversed(SLIDE_DIR_REVERSED != SLIDE_B_REVERSED);
    motorA.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motorA.tare_position();
    motorB.tare_position();

    active = false;
    atTarget = true;
    lastOutMv = 0.0;
    manualMode = false;
    manualMv = 0.0;
    zeroed = false;   // 刚上电，位置不可信
}

void slide::update() {
    // 手动绕开 PID，但限位、过温、slew 都留着。
    // 手动是唯一有人主动往限位里顶的场景，这里更需要保护。
    if (manualMode) {
        // 软限位只在「归零过 + 编码器有数」时才生效。
        // 没归零时位置 0 不代表真的在底部；端口没插电机时 getInches() 是 INFINITY。
        // 这两种情况都会让按键立刻被判成越界、电压被清零，表现就是「按键没反应」。
        const double pos = getInches();
        if (zeroed && std::isfinite(pos)) {
            if ((pos <= SLIDE_MIN_IN && manualMv < 0) || (pos >= SLIDE_MAX_IN && manualMv > 0)) {
                manualMv = 0;
            }
        }
        writeBoth(limited(manualMv));
        return;
    }

    if (!active) {
        return;
    }

    const double err = targetIn - getInches();

    // 误差够小还要稳 80ms，否则停在临界点会反复启停
    if (std::fabs(err) < SLIDE_TOL_IN) {
        settleMs += UPDATE_PERIOD_MS;
        if (settleMs >= 80) {
            brakeBoth();
            active = false;
            atTarget = true;
            return;
        }
    } else {
        settleMs = 0;
    }

    // 超时就刹车退出，不能一直往里怼
    if (pros::millis() - startMs > static_cast<std::uint32_t>(ACTION_TIMEOUT_MS)) {
        brakeBoth();
        active = false;
        atTarget = false;
        return;
    }

    // 重力前馈只在离目标远时给，近了撤掉，否则一路顶过头
    double out = SLIDE_KP_MV_PER_IN * err;
    if (std::fabs(err) > 0.4) {
        out += SLIDE_KG_MV;
    }
    writeBoth(limited(out));
}

void slide::toInchesAsync(double inches) {
    // 越界直接拒绝，不钳制。参数错应该立刻暴露，而不是悄悄停在边界
    if (inches < SLIDE_MIN_IN || inches > SLIDE_MAX_IN) {
        return;
    }
    targetIn = inches;
    active = true;
    atTarget = false;
    settleMs = 0;
    startMs = pros::millis();
}

void slide::toInches(double inches) {
    toInchesAsync(inches);
    while (active) {
        pros::delay(5);
    }
}

void slide::nudge(double deltaIn) {
    toInchesAsync(targetIn + deltaIn);
}

void slide::manual(int mv) {
    if (mv == 0) {
        // 只有「确实在手动」才收尾。这一句挡的是：controls::task 每 20ms 无条件调一次
        // manual(0)，如果不判断状态，它会顺手把 B / X 刚发出去的位姿指令掐掉。
        if (!manualMode) {
            return;
        }
        manualMode = false;
        stop();   // 齿条不自锁，不掉是因为 BRAKE_HOLD 在主动保持
        return;
    }

    manualMode = true;
    manualMv = static_cast<double>(mv);
    active = false;
    atTarget = false;
}


void slide::stop() {
    active = false;
    atTarget = false;
    lastOutMv = 0.0;
    writeBoth(0);
    brakeBoth();
}

void slide::hold() {
    active = false;
    brakeBoth();
}

bool slide::isBusy() { return active; }

bool slide::isAtTarget() { return atTarget; }

bool slide::sensorOk() { return std::isfinite(getInches()); }

bool slide::isZeroed() { return zeroed; }

double slide::getInches() { return motorA.get_position() * SLIDE_IN_PER_DEG; }

double slide::getTemperature() {
    return std::max(motorA.get_temperature(), motorB.get_temperature());
}

void slide::tare() {
    motorA.tare_position();
    motorB.tare_position();
    targetIn = 0.0;
    zeroed = true;   // 从这里开始，位置 0 才是真的 0
}

// 上电时滑轨可能停在中间，位置全错。慢速下到底，堵转处定义为 0。
bool slide::zeroByStall(int timeoutMs) {
    manualMode = false;   // 不清掉，归零结束后 update 会拿旧电压接着顶
    manualMv = 0.0;
    active = false;
    const std::uint32_t t0 = pros::millis();
    std::uint32_t stallMs = 0;

    while (pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
        writeBoth(-3000);
        if (std::fabs(motorA.get_actual_velocity()) < 3.0) {
            stallMs += 20;
            if (stallMs >= 300) {
                writeBoth(0);
                pros::delay(200);
                tare();
                atTarget = true;
                return true;
            }
        } else {
            stallMs = 0;
        }
        pros::delay(20);
    }
    writeBoth(0);
    atTarget = false;
    brakeBoth();
    return false;
}
