// 机构单独调试，底盘没装好或不想开整车时跑这个。
// 把 mech::testUpdate() 放进 opcontrol 的 while 里，比赛前注释掉。
// 换进来之前，先把 main.cpp 里 mech::home() 和 goTo 那两行注释掉，
// 否则上电会自己找限位，参数没标定时就是在瞎撞。
//
// 左摇杆上下 滑轨升降   右摇杆上下 摇臂转动
// A 爪子开合   X 滑轨归零   Y 摇臂归零

#include "mech/mech.hpp"

#include <cmath>

#include "pros/misc.hpp"

namespace {
pros::Controller testCtrl(pros::E_CONTROLLER_MASTER);
}

namespace mech {

void testUpdate() {
    const int ly = testCtrl.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int ry = testCtrl.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // 推得越深每周期加得越多，松手就停在当前位置
    if (std::abs(ly) > 15) {
        slide::nudge(ly / 127.0 * 0.45);
    }
    if (std::abs(ry) > 15) {
        arm::nudge(ry / 127.0 * 1.2);
    }

    if (testCtrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        claw::toggle();
    }
    if (testCtrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        slide::zeroByStall();
    }
    if (testCtrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
        arm::zeroByStall();
    }

    printStatus(7);
}

}  // namespace mech
