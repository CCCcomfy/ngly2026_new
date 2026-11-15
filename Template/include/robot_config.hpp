#pragma once

#include "main.h"

#include "chassis.hpp"
#include "display.hpp"
#include "robot.hpp"

// robot_config —— 车辆设置（车辆信息）
class Config {
public:
    // 电机：左右各两个
    static constexpr int kLeftFrontPort = 4;   // 左前
    static constexpr int kLeftBackPort = 10;    // 左后
    static constexpr int kRightFrontPort = 2;  // 右前
    static constexpr int kRightBackPort = 5;   // 右后

    static constexpr pros::motor_gearset_e_t kDriveGearset = pros::E_MOTOR_GEARSET_18;  // 绿 200rpm
    // 底盘最高转速（rpm），必须和上面的齿轮箱对上：绿 18:1 = 200、红 36:1 = 100、蓝 6:1 = 600。
    static constexpr int kMaxDriveRpm = 200;

    // 电机到轮子的传动比（轮子转速 / 电机转速）。
    static constexpr double kDriveGearRatio = 5.0 / 3.0;

    // 机构
    static constexpr int kLiftPort = 7;
    static constexpr pros::motor_gearset_e_t kLiftGearset = pros::E_MOTOR_GEARSET_36;  // 红 100rpm
    static constexpr char kClampPort = 'A';  // ADI三线口

    // 传感器
    static constexpr int kImuPort = 6;
    
    // 摇杆映射
    static constexpr int kStickDeadband = 8;     // 死区：中位抖动直接吃掉
    static constexpr double kMinOutput = 10.0;   // 刚出死区时的最小输出，用来顶过静摩擦
    static constexpr double kCurveGain = 1.01;   // 曲线强度。1 = 线性，越大中心越"软"
};
extern Chassis g_chassis;
extern Robot g_robot;
extern pros::Controller g_master;

// 主控屏幕。构造顺序有讲究：它持有上面三个对象的引用，必须排在它们后面。
extern Display g_display;
