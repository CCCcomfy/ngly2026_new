// 手动阶段的机构部分：手柄按键 → 滑轨 / 摇臂 / 爪子。
// 不碰底盘。底盘由 main.cpp 的 opcontrol 直接调 g_chassis，两边不要重复。
//
// R2 / R1  摇臂顺时针 / 逆时针（按住）
// L2 / L1  滑轨升 / 降（按住）
// A  爪子开合    B  复原位    X  取件位

#include "mech/controls.hpp"

#include "mech/mech.hpp"
#include "mech/mech_config.hpp"
#include "pros/misc.hpp"

static pros::Controller master(pros::E_CONTROLLER_MASTER);

void controls::task() {

    // 滑轨：两个键都按等于停，而不是后一个覆盖前一个
    int slideMv = 0;
    const bool slideUp = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    const bool slideDn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    if (slideUp && !slideDn) {
        slideMv = SLIDE_MANUAL_UP_MV;
    } else if (slideDn && !slideUp) {
        slideMv = SLIDE_MANUAL_DOWN_MV;
    }
    slide::manual(slideMv);

    // 摇臂：松手回闭环托住，不是断电——断电臂会砸下来
    int armMv = 0;
    const bool armCw = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);   // 顺 = 下压
    const bool armCcw = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);  // 逆 = 上抬
    if (armCw && !armCcw) {
        armMv = ARM_MANUAL_MV;
    } else if (armCcw && !armCw) {
        armMv = -ARM_MANUAL_MV;
    }
    arm::manual(armMv);

    // 爪子：new_press 只在按下那一帧为真，按住不会连发
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        claw::toggle();
    }

    // 复原：滑轨归底 + 摇臂贴回车头 + 爪闭合。行驶前、打完一件都按它
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        mech::goToAsync(mech::TRAVEL);
    }

    // 取件位：爪口摆到竖件高度，顺带张爪，接着按 A 夹
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        mech::goToAsync(mech::GROUND);
    }
}
