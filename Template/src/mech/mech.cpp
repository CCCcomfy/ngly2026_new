// 机构总调度：把滑轨、摇臂、爪子合成一个位姿，上层一句话调用。

#include "mech/mech.hpp"

#include <cstdint>
#include <cstdio>

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace {

pros::Task* ctrlTask = nullptr;

// 非阻塞位姿的状态机。顺序按目标爪态推出来，不改 Pose 结构：
//   要夹紧 -> 先夹，等爪到位，再动两轴；
//   要张开 -> 先动两轴，到位后再松爪，等爪张开。
enum class SeqStage { Idle, WaitClaw, MoveAxes, WaitClawDone };

bool pending = false;
mech::Pose pendingPose{};
SeqStage stage = SeqStage::Idle;

void startAxes(const mech::Pose& p) {
    slide::toInchesAsync(p.slideIn);
    arm::toAngleAsync(p.armDeg);
    stage = SeqStage::MoveAxes;
}

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
        // PROS 3.8 的三参数版本是 (函数, 参数, 名字)，没有独立的栈深度参数。
        ctrlTask = new pros::Task(ctrlLoop, nullptr, "mech");
    }
}

// 控制任务到底起没起来。没起来的话，按键只会改标志、永远没有电机动作。
bool mech::taskRunning() { return ctrlTask != nullptr; }

void mech::home() {
    pending = false;
    stage = SeqStage::Idle;
    slide::zeroByStall();   // 先降到底，摇臂才有转动空间
    arm::zeroByStall();
}

bool mech::goTo(const Pose& p, int timeoutMs) {
    // 越界直接判失败。两个 Async 遇越界是静默返回的，isBusy() 会立刻为 false，
    // 这里就会误判成"已到位"然后去夹爪——夹的是空气，还不报错。
    if (p.slideIn < SLIDE_MIN_IN || p.slideIn > SLIDE_MAX_IN || p.armDeg < ARM_MIN_DEG ||
        p.armDeg > ARM_MAX_DEG) {
        return false;
    }

    // 要夹紧就先夹，等爪真的走完行程再抬臂；否则爪刚动、臂已经起来了。
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

    // 要张开就等两轴到位后再松，避免带着件边走边掉。
    if (!p.clawClosed && claw::isClosed()) {
        claw::release();
    }
    return true;
}

// 爪子用不等版本。主循环每 20ms 跑一次，被 220ms 卡住的话按一次键车会顿一下。
void mech::goToAsync(const Pose& p) {
    if (p.slideIn < SLIDE_MIN_IN || p.slideIn > SLIDE_MAX_IN || p.armDeg < ARM_MIN_DEG ||
        p.armDeg > ARM_MAX_DEG) {
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
        claw::grabNoWait();       // 先夹
    }
    // 爪还在动作（包括上电那 220ms 的初始夹紧）时，等它稳定再动两轴。
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
    arm::stop();   // 不是断电，是托住
}

void mech::printStatus(int row) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "SL %5.1f in   AR %5.0f deg", slide::getInches(),
                  arm::getAngle());
    pros::screen::print(pros::E_TEXT_MEDIUM, row, "%s", buf);
}
