// 双作用缸 + 5/2 单电控阀，断电回默认位
// 唯一要小心的是发指令到真正夹紧有时间差，这是"偶尔掉件"的来源。

#include "mech/mech.hpp"

#include "pros/adi.hpp"
#include "pros/rtos.hpp"

namespace {

// PROS 3.8 里类名是 pros::ADIDigitalOut，没有 pros::adi:: 命名空间。
pros::ADIDigitalOut solenoid(PORT_CLAW_SOL);


bool closed = false;

}  // namespace

void claw::init() {
    // 上电先张开。上次断电时可能还夹着件，直接夹紧会夹碎或夹到自己结构
    solenoid.set_value(false);
    closed = false;
}

// 只发指令立刻返回。手柄别直接用，气缸没走完行程就抬臂会把件甩掉。
void claw::grabNoWait() {
    solenoid.set_value(true);
    closed = true;
}

void claw::releaseNoWait() {
    solenoid.set_value(false);
    closed = false;
}

// 180ms = 阀芯 20~30 + 气管充气 50~100 + 活塞行程 50~80 + 爪片贴合 10~20。
// 宁可 200 不要 120：多等 0.02 秒是零头，掉一个件是整条路线报废。
void claw::grab() {
    grabNoWait();
    pros::delay(CLAW_ACT_MS);
}

void claw::release() {
    releaseNoWait();
    pros::delay(CLAW_ACT_MS);   // 松开也要等，没张开就走位会被带着偏
}

// 这 180ms 里主循环卡住，底盘收不到新指令，按一下会顿一下。低速无感，先这样。
void claw::toggle() {
    if (closed) {
        release();
    } else {
        grab();
    }
}

bool claw::isClosed() { return closed; }
