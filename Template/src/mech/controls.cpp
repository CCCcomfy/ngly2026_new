
#include "mech/controls.hpp"

#include "input/input.hpp"
#include "mech/mech.hpp"
#include "mech/mech_config.hpp"
#include "pros/misc.hpp"

void controls::task()
{

    int slideMv = 0;
    const bool slideUp = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool slideDn = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (slideUp && !slideDn)
    {
        slideMv = SLIDE_MANUAL_UP_MV;
    }
    else if (slideDn && !slideUp)
    {
        slideMv = SLIDE_MANUAL_DOWN_MV;
    }
    slide::manual(slideMv);

    int armMv = 0;
    const bool armUp = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);   // 上抬
    const bool armDown = g_master.get_digital(pros::E_CONTROLLER_DIGITAL_R2); // 下压
    if (armUp && !armDown)
    {
        armMv = ARM_MANUAL_MV;
    }
    else if (armDown && !armUp)
    {
        armMv = -ARM_MANUAL_MV;
    }
    arm::manual(armMv);

    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        claw::toggleNoWait();
    }

    // 复原
    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        mech::goToAsync(mech::TRAVEL);
    }
    // 没时间调参，b和x键都不敢使用，就此作废
    //  取件位
    if (g_master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        mech::goToAsync(mech::GROUND);
    }
}
