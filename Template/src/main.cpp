#include "main.h"

#include "chassis.hpp"
#include "robot.hpp"
#include "robot_config.hpp"

/**
 * 只执行一次的入场准备。执行期间所有比赛模式都会被挡住，所以要快。
 */
void initialize() {
    pros::lcd::initialize();
    g_display.draw("[BOOT]");

    // 阻塞式校准（约 2 秒）：这一行返回之前车必须完全静止。
    // 否则 IMU 会把「启动时的晃动」当成零位，之后所有转向都是歪的。
    g_chassis.resetImu();

    g_chassis.initialize();
    g_robot.initialize();
    g_display.draw("[READY]");
}

/**
 * 机器人处于「被禁用」状态时运行：赛前待机、自动结束到手动开始之间、断联之后。
 * 这里是唯一适合做「安全动作」的地方 —— 让所有机构停下来。
 */
void disabled() {
    g_chassis.stop();
    g_robot.liftStop();
    g_display.draw("[DISABLED]");
}

/**
 * 赛前钩子：只在接了比赛控制时运行，用来选自动方案、调参数。
 * 现在只有一套自动，所以留空；以后要做红 / 蓝 / AWP 选择就写在这里。
 */
void competition_initialize() {
}

/**
 * 自动阶段。跑在独立任务里，被禁用或断联时会被强行掐断。
 * 必须可重入：每次进来都当成脏状态，先归零再跑路径。
 */
void autonomous() {
    g_chassis.initialize();   // 编码器清零 + 刹车模式
    g_chassis.tareHeading();  // 朝向归零
    g_display.draw("[AUTO]");
}

// 手动赛
void opcontrol() {
    g_display.draw("[OPCTRL]");

    int loop_count = 0;

    while (true) {
        const int left = g_master.get_analog(ANALOG_LEFT_Y);
        const int right = g_master.get_analog(ANALOG_RIGHT_Y);
        g_chassis.drive(left, right);

        // 屏幕约 4Hz 刷新。写屏比想象中慢，每圈都画会把主循环拖到十几赫兹，
        // 表现就是「摇杆一动车要慢半拍」。
        if (++loop_count % 12 == 0) {
            g_display.draw("[OPCTRL]");
        }

        pros::delay(20);
    }
}
