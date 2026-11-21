// 手动阶段的机构部分：手柄按键 → 滑轨 / 摇臂 / 爪子。
// 不碰底盘。底盘由 main.cpp 的 opcontrol 直接调 g_chassis，两边不要重复。
//
// R2 / R1  摇臂顺时针 / 逆时针（按住）
// L2 / L1  滑轨升 / 降（按住）
// A  爪子开合    B  复原位    X  取件位

#include "mech/controls.hpp"

#include "chassis/chassis_config.hpp"   // 用全局 g_master，别再建第二个 Controller
#include "mech/mech.hpp"
#include "mech/mech_config.hpp"
#include "pros/misc.hpp"

void controls::task() {

    // 滑轨：两个键都按等于停，而不是后一个覆盖前一个
    int slideMv = 0;
    const bool slideUp = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool slideDn = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (slideUp && !slideDn) {
        slideMv = SLIDE_MANUAL_UP_MV;
    } else if (slideDn && !slideUp) {
        slideMv = SLIDE_MANUAL_DOWN_MV;
    }
    slide::manual(slideMv);

    // 摇臂：松手回闭环托住，不是断电——断电臂会砸下来。
    // 正电压 = 往上托（重力前馈就是这个方向），负电压 = 往下压，
    // 归零 zeroByStall 也是写 -3000 往下顶，用的是同一个约定。
    int armMv = 0;
    const bool armUp = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);    // 上抬
    const bool armDown = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);  // 下压
    if (armUp && !armDown) {
        armMv = ARM_MANUAL_MV;
    } else if (armDown && !armUp) {
        armMv = -ARM_MANUAL_MV;
    }
    arm::manual(armMv);

    // 爪子：new_press 只在按下那一帧为真，按住不会连发
    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        claw::toggleNoWait();
    }

    // 复原：滑轨归底 + 摇臂贴回车头 + 爪闭合。行驶前、打完一件都按它
    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        mech::goToAsync(mech::TRAVEL);
    }

    // 取件位：爪口摆到竖件高度，顺带张爪，接着按 A 夹
    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        mech::goToAsync(mech::GROUND);
    }
}
