#pragma once

#include "chassis/chassis.hpp"
#include "main.h"

// Display —— 主控屏幕
class Display
{
public:
    Display(Chassis &chassis, pros::Controller &master);

    void draw(const char *mode);

    // 把「上电以来的最高轮速」清零，方便反复测
    void resetPeak();

private:
    Chassis &m_chassis;
    pros::Controller &m_master;

    double m_peak_wheel_rpm; // 上电以来见过的最高轮速（绝对值）
};

extern Display g_display;