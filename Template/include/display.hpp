#pragma once

#include "chassis.hpp"
#include "main.h"
#include "robot.hpp"

// Display —— 主控屏幕
class Display {
public:
    Display(Chassis& chassis, Robot& robot, pros::Controller& master);

    void draw(const char* mode);

    // 把「上电以来的最高轮速」清零，方便反复测
    void resetPeak();

private:
    Chassis& m_chassis;
    Robot& m_robot;
    pros::Controller& m_master;

    double m_peak_wheel_rpm;  // 上电以来见过的最高轮速（绝对值）
};
