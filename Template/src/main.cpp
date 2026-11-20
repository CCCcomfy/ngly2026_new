#include "main.h"

#include "chassis/chassis.hpp"
#include "mech/controls.hpp"
#include "mech/mech.hpp"
#include "core/robot_config.hpp"

/**
 * 只执行一次的入场准备。执行期间所有比赛模式都会被挡住，所以要快。
 */
void initialize()
{
    pros::lcd::initialize();
    g_display.draw("[BOOT]");

    // 阻塞式校准（约 2 秒）：这一行返回之前车必须完全静止。
    // 否则 IMU 会把「启动时的晃动」当成零位，之后所有转向都是歪的。
    g_chassis.resetImu();

    g_chassis.initialize();

    // 机构上电：先起控制任务，再归零（滑轨降到堵转、摇臂压到限位），
    // 最后收到行驶位。归零两轴各最多 3 秒，参数没标定时会一直撞，
    // 调试期先注释掉 home 和 goTo 这两行。
    mech::init();
    // mech::home();
    // mech::goTo(mech::TRAVEL, 1500);

    g_display.draw("[READY]");
}

/**
 * 机器人处于「被禁用」状态时运行：赛前待机、自动结束到手动开始之间、断联之后。
 * 这里是唯一适合做「安全动作」的地方 —— 让所有机构停下来。
 */
void disabled()
{
    g_chassis.stop();
    mech::emergencyStop(); // 滑轨刹车、摇臂原地托住，不是断电
    g_display.draw("[DISABLED]");
}

/**
 * 赛前钩子：只在接了比赛控制时运行，用来选自动方案、调参数。
 * 现在只有一套自动，所以留空；以后要做红 / 蓝 / AWP 选择就写在这里。
 */
void competition_initialize()
{
}

/**
 * 自动阶段。跑在独立任务里，被禁用或断联时会被强行掐断。
 * 必须可重入：每次进来都当成脏状态，先归零再跑路径。
 */
void autonomous()
{
    g_chassis.initialize();  // 编码器清零 + 刹车模式
    g_chassis.tareHeading(); // 朝向归零
    g_display.draw("[AUTO]");

    g_chassis.setPose(0, 0, 0);
    g_chassis.driveDistance(1.28, 2000);
    g_chassis.turnTo(90, 2000);
    g_chassis.driveDistance(1, 2000);
    mech::goTo(mech::LOW_GOAL, 2000); // 放初始pin

    g_chassis.driveDistance(-1, 2000);
    g_chassis.turnTo(45, 2000);
    mech::goTo(mech::GROUND, 2000);
    g_chassis.driveDistance(2.5, 5000);
    claw::grab();

    mech::goTo(mech::CARRY, 2000);
    g_chassis.driveDistance(-1.25, 2000);
    g_chassis.turnTo(135, 2000);
    g_chassis.driveDistance(1.25, 2000);
    mech::goTo(mech::STACK, 2000);
    claw::release();

    mech::goTo(mech::TRAVEL, 2000);
    g_chassis.driveDistance(-1.25, 2000);
    g_chassis.turnTo(225, 2000);
    g_chassis.driveDistance(1.25, 2000);
    g_chassis.turnTo(0, 2000);
    g_chassis.driveDistance(-1.28, 2000);

    g_chassis.driveDistance(0.5, 2000);

    g_chassis.stop();
}

// 手动赛
void opcontrol()
{
    g_display.draw("[OPCTRL]");

    int loop_count = 0;

    while (true)
    {
        // 街机式：左杆前后，右杆左右转向（右推右转）
        const int power = g_master.get_analog(ANALOG_LEFT_Y);
        const int turn = g_master.get_analog(ANALOG_RIGHT_X);
        g_chassis.arcade(power, turn);

        // 机构手动控制：滑轨 L2/L1、摇臂 R2/R1、爪 A、复原 B、取件 X。
        // 不阻塞，必须在每圈里调；底盘那两行不要重复处理手柄。
        controls::task();

        // 屏幕约 4Hz 刷新。写屏比想象中慢，每圈都画会把主循环拖到十几赫兹，
        // 表现就是「摇杆一动车要慢半拍」。
        if (++loop_count % 12 == 0)
        {
            g_display.draw("[OPCTRL]");
        }

        pros::delay(20);
    }
}
