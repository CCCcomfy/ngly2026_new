// 机构总调度：把滑轨、摇臂、爪子合成一个位姿，上层一句话调用。

#include "mech/mech.hpp"

#include <cstdint>
#include <cstdio>

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace {

pros::Task* ctrlTask = nullptr;

void ctrlLoop(void*) {
    while (true) {
        slide::update();
        arm::update();
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

void mech::home() {
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

    slide::toInchesAsync(p.slideIn);
    arm::toAngleAsync(p.armDeg);

    const std::uint32_t t0 = pros::millis();
    while ((slide::isBusy() || arm::isBusy()) &&
           pros::millis() - t0 < static_cast<std::uint32_t>(timeoutMs)) {
        pros::delay(5);
    }

    const bool ok = !slide::isBusy() && !arm::isBusy();
    if (!ok) {
       return false;
    }

    // 爪子最后动。状态已经一样就跳过，一次省 180ms
    if (p.clawClosed != claw::isClosed()) {
        if (p.clawClosed) {
            claw::grab();
        } else {
            claw::release();
        }
    }
    return ok;
}

// 爪子用不等版本。主循环每 20ms 跑一次，被 180ms 卡住的话按一次键车会顿一下。
void mech::goToAsync(const Pose& p) {

    if (p.clawClosed != claw::isClosed()) {
        if (p.clawClosed) {
            claw::grab();
        } else {
            claw::releaseNoWait();
        }
    }
        slide::toInchesAsync(p.slideIn);
    arm::toAngleAsync(p.armDeg);


    }


bool mech::isBusy() { return slide::isBusy() || arm::isBusy(); }

void mech::emergencyStop() {
    slide::stop();
    arm::stop();   // 不是断电，是托住
}

void mech::printStatus(int row) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "SL %5.1f in   AR %5.0f deg", slide::getInches(),
                  arm::getAngle());
    pros::screen::print(pros::E_TEXT_MEDIUM, row, "%s", buf);
}
