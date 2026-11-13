#pragma once

#include "main.h"

#include "chassis.hpp"
#include "robot.hpp"

// robot_config —— 车辆设置（车辆信息）
class Config {
public:
    // 电机：左右各三个
    static constexpr int kLeftFrontPort = 1;   // 左前
    static constexpr int kLeftMidPort = 2;     // 左中
    static constexpr int kLeftBackPort = 3;    // 左后
    static constexpr int kRightFrontPort = 4;  // 右前
    static constexpr int kRightMidPort = 5;    // 右中
    static constexpr int kRightBackPort = 6;   // 右后

    static constexpr pros::motor_gearset_e_t kDriveGearset = pros::E_MOTOR_GEARSET_18;  // 绿 200rpm

    // 机构
    static constexpr int kLiftPort = 7;
    static constexpr pros::motor_gearset_e_t kLiftGearset = pros::E_MOTOR_GEARSET_36;  // 红 100rpm
    static constexpr char kClampPort = 'A';  // ADI三线口

    // 传感器
    static constexpr int kImuPort = 8;
};
extern Chassis g_chassis;
extern Robot g_robot;
extern pros::Controller g_master;
