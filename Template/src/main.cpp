#include "main.h"

#include "chassis/chassis.hpp"
#include "mech/controls.hpp"
#include "mech/mech.hpp"
#include "chassis/chassis_config.hpp"
#include "ui/debug.hpp"

/**
 * 只执行一次的入场准备。执行期间所有比赛模式都会被挡住，所以要快。
 */
void initialize()
{
    pros::lcd::initialize();
    g_display.draw("[BOOT]");

    // 机构上电必须排在 IMU 校准之前：mech::init() 起的是独立的 10ms 控制任务，
    // 放在阻塞校准后面的话，IMU 一卡住（没接、校准超时）机构就永远没有控制任务，
    // 表现就是「按键毫无反应」。机构先起，底盘再校准，互不阻塞。
    mech::init();

    // 阻塞式校准（约 2 秒）：这一行返回之前车必须完全静止。
    // 否则 IMU 会把「启动时的晃动」当成零位，之后所有转向都是歪的。
    g_chassis.resetImu();

    g_chassis.initialize();

    // 手柄调试屏：起一个 5Hz 的后台任务，自动赛里不用再管它画屏
    dbg::start(g_master);

    // 归零：滑轨降到堵转、摇臂压到限位，最后收到行驶位。
    // 两轴各最多 3 秒，参数没标定时会一直撞，调试期先注释掉 home 和 goTo 这两行。
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
 *
 * 调试：手柄屏第 0 行显示当前步骤名，超时时变成 "!"（步骤名保留），
 * 第 2 行是这一步的实时误差，调参数时主要看它能不能收敛到 0 附近。
 */
void autonomous()
{
    g_chassis.initialize();  // 编码器清零 + 刹车模式
    g_chassis.tareHeading(); // 朝向归零
    g_display.draw("[AUTO]");

    g_chassis.setPose(0, 0, 0);

    dbg::step("PIN"); // 放初始 pin
    g_chassis.driveDistance(26.4, 2000);
    g_chassis.turnTo(90, 2000);
    g_chassis.driveDistance(22, 2000);
    //mech::goTo(mech::LOW_GOAL, 2000);

    //dbg::step("GRAB"); // 退出来取第一个块
    g_chassis.driveDistance(-22, 2000);
    g_chassis.turnTo(45, 2000);
    //mech::goTo(mech::GROUND, 2000);
    g_chassis.driveDistance(55, 5000);
    //claw::grab();
    //mech::goTo(mech::CARRY, 2000);

    //dbg::step("STACK"); // 回场地中间堆叠
    g_chassis.driveDistance(-55, 2000);
    g_chassis.turnTo(90, 2000);
    g_chassis.driveDistance(22, 2000);
    //mech::goTo(mech::STACK, 2000);
    //claw::release();
    //mech::goTo(mech::TRAVEL, 2000);

    //dbg::step("HOME"); // 收车
    g_chassis.driveDistance(-22, 2000);
    g_chassis.turnTo(0, 2000);
    g_chassis.driveDistance(-28, 2000);
    g_chassis.driveDistance(5, 2000);

    dbg::step("DONE");
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
