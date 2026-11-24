
#include "mech/mech.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "pros/motors.hpp"
#include "pros/rtos.hpp"
/*之前初版使用cos重力前馈，但因为相关参数来不调和摇臂重力力矩过大而放弃
改用pid控制，在负值和正20度之间给满压12V，超过20度再收紧


*/
namespace {

pros::Motor motorA(PORT_ARM_A, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor motorB(PORT_ARM_B, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

struct PidState {
    double integral = 0.0;
    double prevMeas = 0.0;
    bool hasPrev = false;

    void reset() {
        integral = 0.0;
        prevMeas = 0.0;
        hasPrev = false;
    }
};

double targetDeg = 0.0;
double offsetDeg = 0.0;  
bool active = false;      
bool holding = false;     
bool atTarget = true;
bool manualMode = false;
bool stopping = false;    // 松手后先由速度环刹到0，再切位置保持
bool zeroed = false;      // 是否已有可信的真实角度

double lastOutMv = 0.0;  
double lastCmdMv = 0.0;   // 给屏幕读数
double biasMv = 0.0;      // 到位时学到的保持偏置（代替重力 cos 前馈）

double manualMv = 0.0;
double manualVelTarget = 0.0;

PidState posPid;
PidState velPid;

double lastVel = 0.0;
double lastDeg = 0.0;
std::uint32_t lastDegMs = 0;
std::uint32_t negStallMs = 0;
bool hasLastDeg = false;

std::uint32_t startMs = 0;
std::uint32_t settleMs = 0;

constexpr double kDt = UPDATE_PERIOD_MS / 1000.0;

inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 通用 PID
// 输出顶到限幅且误差还在往同方向推时，不再积分。
inline double pidStep(PidState& s, double err, double meas, double dt,
                      double kp, double ki, double kd, double ff,
                      double lo, double hi) {
    if (!s.hasPrev) {
        s.prevMeas = meas;
        s.hasPrev = true;//保存测量历史
    }
    const double deriv = (meas - s.prevMeas) / dt;
    double integ = s.integral + err * dt;
    double out = ff + kp * err + ki * integ - kd * deriv;

    if (out > hi) {
        out = hi;
        if (err > 0.0) {
            integ = s.integral;
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

inline double rawDeg() {
    return motorA.get_position() / ARM_MOTOR_DEG_PER_ARM_DEG;
}

inline void writeBoth(double mv) {
    if (!std::isfinite(mv)) {
        mv = 0.0;
    }
    const int32_t v = static_cast<int32_t>(mv);
    lastCmdMv = mv;
    motorA.move_voltage(v);
    motorB.move_voltage(v);
}

// 用角度差分算角速度，而不是取均值，防止有反向的电机
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

// 温控
// 手动上抬到二十度前用到 12V，其余场合仍是 ARM_MAX_MV。
inline double limited(double mv, double baseCap = ARM_MAX_MV) {
    if (!std::isfinite(mv)) {
        mv = 0.0;
    }
    const double temp = arm::getTemperature();
    const double cap =
        (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? baseCap * 0.5 : baseCap;
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

    // 上电时物理角度负八十度，电机编码器此刻为 0。
    
    offsetDeg = -ARM_BOOT_DEG;
    targetDeg = ARM_BOOT_DEG;

    active = false;
    holding = false;
    atTarget = true;
    manualMode = false;
    stopping = false;
    lastOutMv = 0.0;
    lastCmdMv = 0.0;
    biasMv = 0.0;
    manualMv = 0.0;
    manualVelTarget = 0.0;

    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    lastDeg = 0.0;
    negStallMs = 0;
    settleMs = 0;

  
    zeroed = true;
}

void arm::update() {
  
    if (manualMode || stopping) {
        const double deg = getAngle();
        if (!std::isfinite(deg)) {
            manualMode = false;
            stopping = false;
            active = false;
            holding = false;
            atTarget = false;
            manualVelTarget = 0.0;
            lastOutMv = 0.0;
            writeBoth(0);
            return;
        }

        const double vel = updateVelocity(deg);
        double targetVel = manualMode ? manualVelTarget : 0.0;

        //限位
        if (zeroed && ARM_SOFT_LIMIT_ENABLED) {
            if (deg <= ARM_MIN_DEG && targetVel < 0.0) {
                targetVel = 0.0;
            } else if (deg >= ARM_MAX_DEG && targetVel > 0.0) {
                targetVel = 0.0;
            }
        }

       
        const double ff = manualMode ? manualMv : 0.0;

        
        // 越过正20度后接pid防止过头
        const bool liftBoost =
            (targetVel > 0.0 && deg <= ARM_MANUAL_LIFT_BOOST_DEG);
        const double baseCap = liftBoost ? ARM_MANUAL_LIFT_MAX_MV : ARM_MAX_MV;
        const double temp = getTemperature();
        const double cap =
            (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? baseCap * 0.5 : baseCap;

        double out = 0.0;
        if (manualMode && liftBoost) {
           
            out = cap;
        } else {
            // 刹车阶段不加积分，那一段误差一直是负的，积分会把刹车越刹越狠。
            const double ki = manualMode ? ARM_MANUAL_KI_MV_PER_DPS_SEC : 0.0;
            out = pidStep(velPid, targetVel - vel, vel, kDt,
                          ARM_MANUAL_KP_MV_PER_DPS, ki, 0.0,
                          ff, -cap, cap);
        }

        // 限位处只允许托住，不允许继续往外顶，软限位关闭时不做这段。
        if (zeroed && ARM_SOFT_LIMIT_ENABLED) {
            if (deg <= ARM_MIN_DEG && out < 0.0) {
                out = 0.0;
            } else if (deg >= ARM_MAX_DEG && out > 0.0) {
                out = 0.0;
            }
        }

        writeBoth(limited(out, baseCap));

        // 松手后速度降到阈值才切位置保持，防止突然跳回零掉臂
        if (stopping && std::fabs(vel) < ARM_STOP_DPS) {
            stopping = false;
            targetDeg = deg;
            active = false;
            holding = true;
            atTarget = true;
            biasMv = 0.0;
            posPid.reset();
            velPid.reset();
            hasLastDeg = false;
            lastVel = 0.0;
            negStallMs = 0;
        }
        return;
    }

    // 位置环
    if (!active && !holding) {
        return;
    }

    const double deg = getAngle();
    if (!std::isfinite(deg)) {
        manualMode = false;
        stopping = false;
        active = false;
        holding = false;
        atTarget = false;
        lastOutMv = 0.0;
        writeBoth(0);
        return;
    }

    const double err = targetDeg - deg;
    updateVelocity(deg);

    if (active) {
        // 到误差带内还要稳 80ms，避免在临界点反复启停
        if (std::fabs(err) < ARM_TOL_DEG) {
            settleMs += UPDATE_PERIOD_MS;
            if (settleMs >= 80) {
                active = false;
                holding = true;
                atTarget = true;
                // 把到位时的实际输出记成保持偏置，维持稳定
                biasMv = clampd(lastOutMv, -ARM_MAX_MV, ARM_MAX_MV);
                posPid.reset();
            }
        } else {
            settleMs = 0;
        }

        // 超时不能刹车，一松臂就砸，要改成原地托住，目标改当前角度。
        if (active && pros::millis() - startMs >
                          static_cast<std::uint32_t>(ACTION_TIMEOUT_MS)) {
            targetDeg = deg;
            active = false;
            holding = true;
            atTarget = false;
            biasMv = 0.0;
            posPid.reset();
            return;
        }
    }

    const double temp = getTemperature();
    const double cap =
        (!std::isfinite(temp) || temp > TEMP_LIMIT_C) ? ARM_MAX_MV * 0.5 : ARM_MAX_MV;

    // 水平位靠积分补托举。
    const double out = pidStep(posPid, err, deg, kDt,
                               ARM_KP_MV_PER_DEG, ARM_KI_MV_PER_DEG_SEC,
                               ARM_KD_MV_PER_DEG_PER_SEC, biasMv,
                               -cap, cap);
    writeBoth(limited(out));
}

void arm::toAngleAsync(double deg) {
    if (!std::isfinite(deg) ||
        (ARM_SOFT_LIMIT_ENABLED && (deg < ARM_MIN_DEG || deg > ARM_MAX_DEG))) {
        return;
    }

    targetDeg = deg;
    active = true;
    holding = false;
    atTarget = false;
    manualMode = false;
    stopping = false;

    biasMv = 0.0;
    settleMs = 0;
    startMs = pros::millis();
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    negStallMs = 0;
}

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
            return;
        }
        
        manualMode = false;
        stopping = true;
        manualVelTarget = 0.0;
        negStallMs = 0;
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
        negStallMs = 0;
    }

    active = false;
    holding = false;
    atTarget = false;
    biasMv = 0.0;
}

void arm::stop() {
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
    biasMv = 0.0;
    manualVelTarget = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    negStallMs = 0;
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
    active = false;
    holding = true;
    atTarget = true;
    biasMv = 0.0;
    manualVelTarget = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    negStallMs = 0;
}

bool arm::isBusy() {
    return active;
}

bool arm::isAtTarget() {
    return atTarget;
}

bool arm::sensorOk() {
    return std::isfinite(getAngle());
}

bool arm::isZeroed() {
    return zeroed;
}

double arm::lastCommandMv() {
    return lastCmdMv;
}

double arm::getAngle() {
    return rawDeg() - offsetDeg;
}

double arm::getTemperature() {
    return std::max(motorA.get_temperature(), motorB.get_temperature());
}

void arm::tare() {
   
    const double raw = rawDeg();
    if (!std::isfinite(raw)) {
        return;
    }
    offsetDeg = raw;
    targetDeg = 0.0;
    zeroed = true;
    biasMv = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    negStallMs = 0;
}

// 慢速往下压，撞到机械限位后速度归零，把那个位置定义成ARM_MIN_DEG。
bool arm::zeroByStall(int timeoutMs) {
    manualMode = false;
    stopping = false;
    manualMv = 0.0;
    manualVelTarget = 0.0;
    active = false;
    holding = false;
    lastOutMv = 0.0;
    biasMv = 0.0;
    posPid.reset();
    velPid.reset();
    hasLastDeg = false;
    lastVel = 0.0;
    negStallMs = 0;

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
                zeroed = true;
                biasMv = 0.0;
                lastOutMv = 0.0;
                return true;
            }
        } else {
            stallMs = 0;
        }
        pros::delay(20);
    }

    writeBoth(0);
    const double deg = getAngle();
    if (std::isfinite(deg)) {
        targetDeg = deg;
        holding = true;
    } else {
        holding = false;
    }
    biasMv = 0.0;
    lastOutMv = 0.0;
    atTarget = false;
    return false;
}
