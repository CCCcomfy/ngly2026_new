// 机构总调度

#include "mech/mech.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace {

pros::Task* ctrlTask = nullptr;

//状态机，规定爪子和臂动的顺序
enum class SeqStage { Idle, WaitClaw, MoveAxes, WaitClawDone };

bool pending = false;
mech::Pose pendingPose{};
SeqStage stage = SeqStage::Idle;

void startAxes(const mech::Pose& p) {
    slide::toInchesAsync(p.slideIn);
    arm::toAngleAsync(p.armDeg);
    stage = SeqStage::MoveAxes;
}
//状态机心跳
void sequencerTick() {
    if (!pending) {
        return;
    }

    switch (stage) {
        case SeqStage::WaitClaw:
            if (claw::isSettled()) {
                startAxes(pendingPose);
            }
            break;

        case SeqStage::MoveAxes:
            if (!slide::isBusy() && !arm::isBusy()) {
                if (!pendingPose.clawClosed && claw::isClosed()) {
                    claw::releaseNoWait();
                    stage = SeqStage::WaitClawDone;
                } else {
                    pending = false;
                    stage = SeqStage::Idle;
                }
            }
            break;

        case SeqStage::WaitClawDone:
            if (claw::isSettled()) {
                pending = false;
                stage = SeqStage::Idle;
            }
            break;

        default:
            break;
    }
}
//反馈的控制环
void ctrlLoop(void*) {
    while (true) {
        slide::update();
        arm::update();
        sequencerTick();
        pros::delay(UPDATE_PERIOD_MS);
    }
}

}  // namespace

void mech::init() {
    claw::init();   // 先关阀，避免上电瞬间爪子乱动
    slide::init();
    arm::init();
    startTask();
}

void mech::startTask() {
    if (ctrlTask == nullptr) {
       
        ctrlTask = new pros::Task(ctrlLoop, nullptr, "mech");
    }
}

// 控制任务到底起没起来。没起来的话，按键只会改标志，永远没有电机动作
bool mech::taskRunning() { return ctrlTask != nullptr; }

void mech::home() {
    pending = false;
    stage = SeqStage::Idle;
    slide::zeroByStall();   // 先降到底，摇臂才有转动空间
    arm::zeroByStall();
}


bool mech::goTo(const Pose& p, int timeoutMs) {
    //先取消可能还在跑的异步位姿
    pending = false;
    stage = SeqStage::Idle;

    // 越界直接判失败
    
    if (p.slideIn < SLIDE_MIN_IN || p.slideIn > SLIDE_MAX_IN ||
        (ARM_SOFT_LIMIT_ENABLED && (p.armDeg < ARM_MIN_DEG || p.armDeg > ARM_MAX_DEG))) {
        return false;
    }

    // 先夹
    if (p.clawClosed && !claw::isClosed()) {
        claw::grab();
    }
    while (!claw::isSettled()) {
        pros::delay(5);
    }

    slide::toInchesAsync(p.slideIn);
    arm::toAngleAsync(p.armDeg);

    const std::uint32_t t0 = pros::millis();
    while ((slide::isBusy() || arm::isBusy()) &&
           pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
        pros::delay(5);
    }

    const bool ok = !slide::isBusy() && !arm::isBusy();
    if (!ok) {
        emergencyStop();
        return false;
    }

    // 避免带着件边走边掉
    if (!p.clawClosed && claw::isClosed()) {
        claw::release();
    }
    return true;
}

bool mech::goTo(double degree, int timeoutMs) {
    if (!std::isfinite(degree) ||
        (ARM_SOFT_LIMIT_ENABLED && (degree < ARM_MIN_DEG || degree > ARM_MAX_DEG))) {
        return false;  
    }

    pending = false;
    stage = SeqStage::Idle;
    arm::toAngleAsync(degree);

    const std::uint32_t t0 = pros::millis();
    while (arm::isBusy() &&
           pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
        pros::delay(5);
    }

    if (arm::isBusy()) {
        arm::stop();   // 超时，原地托住当前角度
        return false;
    }
    return true;      
}


void mech::goToAsync(const Pose& p) {
    if (p.slideIn < SLIDE_MIN_IN || p.slideIn > SLIDE_MAX_IN ||
        (ARM_SOFT_LIMIT_ENABLED && (p.armDeg < ARM_MIN_DEG || p.armDeg > ARM_MAX_DEG))) {
        return;
    }

    if (pending || slide::isBusy() || arm::isBusy()) {
        slide::stop();
        arm::stop();
    }

    pendingPose = p;
    pending = true;
    stage = SeqStage::Idle;

    if (p.clawClosed && !claw::isClosed()) {
        claw::grabNoWait();       
    }
    // 爪还在动作时，等它稳定再动两轴。
    if (!claw::isSettled()) {
        stage = SeqStage::WaitClaw;
    } else {
        startAxes(p);
    }
}


bool mech::isBusy() { return pending || slide::isBusy() || arm::isBusy(); }

void mech::emergencyStop() {
    pending = false;
    stage = SeqStage::Idle;
    slide::stop();
    arm::stop();   //托住
}

void mech::printStatus(int row) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "SL %5.1f in   AR %5.0f deg", slide::getInches(),
                  arm::getAngle());
    pros::screen::print(pros::E_TEXT_MEDIUM, row, "%s", buf);
}
