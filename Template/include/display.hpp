#pragma once

#include "chassis.hpp"
#include "main.h"
#include "robot.hpp"

// Display —— 主控屏幕
//
// 只负责「把状态显示出来」：读 Chassis / Robot 的公开接口，不碰私有成员，
// 也不改任何机器人状态。想加一行信息，只改这个类，main.cpp 不用动。
//
// 注意：主控 LLEMU 的字体不含中文，屏幕上写中文会变成方块 / 乱码，所以文字用英文。
class Display {
public:
    Display(Chassis& chassis, Robot& robot, pros::Controller& master);

    // 重画整屏。主循环里约 5Hz 调一次就够 —— 写屏比想象中慢，
    // 每圈都画会把 50Hz 的主循环拖到十几赫兹。
    void draw(const char* mode);

private:
    Chassis& m_chassis;
    Robot& m_robot;
    pros::Controller& m_master;
};
