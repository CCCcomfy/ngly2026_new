
#include "mech/mech.hpp"

#include "pros/adi.hpp"
#include "pros/rtos.hpp"

namespace {


pros::ADIDigitalOut solenoid(PORT_CLAW_SOL);


bool closed = false;
std::uint32_t settleUntil = 0;   // 上一次开合动作的预计到位时刻


void apply(bool close) {
    if (close == closed) {
        return;
    }
    solenoid.set_value(close);//输出电平让它松开
    closed = close;
    settleUntil = pros::millis() + CLAW_ACT_MS;
}

}  

void claw::init() {
    // 上电夹紧
    closed = false;
    apply(true);
}


void claw::grabNoWait() {
    apply(true);
}

void claw::releaseNoWait() {
    apply(false);
}

// A 键按下翻转当前状态，不阻塞主循环。
void claw::toggleNoWait() {
    apply(!closed);
}


void claw::grab() {
    grabNoWait();
    pros::delay(CLAW_ACT_MS);
}

void claw::release() {
    releaseNoWait();
    pros::delay(CLAW_ACT_MS);   
}

// 这 180ms，按一下会顿一下，但是夹的时候一般不动，没事
void claw::toggle() {
    if (closed) {
        release();
    } else {
        grab();
    }
}

bool claw::isClosed() { return closed; }

bool claw::isSettled() {
    return static_cast<std::int32_t>(pros::millis() - settleUntil) >= 0;
}
