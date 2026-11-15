#pragma once

#include "chassis.hpp"
#include "main.h"
#include "robot.hpp"

// Display —— 主控屏幕
class Display {
public:
    Display(Chassis& chassis, Robot& robot, pros::Controller& master);

    void draw(const char* mode);

private:
    Chassis& m_chassis;
    Robot& m_robot;
    pros::Controller& m_master;
};
