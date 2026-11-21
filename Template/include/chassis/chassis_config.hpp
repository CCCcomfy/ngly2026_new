#pragma once

#include "main.h"

#include "chassis/chassis.hpp"
#include "ui/display.hpp"

// chassis_config —— 底盘与车辆设置（端口、传动比、闭环参数、摇杆曲线）
// 改车之后这里要跟着改的，基本只有端口和齿比两项。
class Config {
public:
    // 电机：左右各两个
    static constexpr int kLeftFrontPort = 7;   // 左前
    static constexpr int kLeftBackPort = 10;    // 左后
    static constexpr int kRightFrontPort = 5;  // 右前
    static constexpr int kRightBackPort = 16;   // 右后

    static constexpr pros::motor_gearset_e_t kDriveGearset = pros::E_MOTOR_GEARSET_18;  // 绿 200rpm
    // 底盘最高转速（rpm），必须和上面的齿轮箱对上：绿 18:1 = 200、红 36:1 = 100、蓝 6:1 = 600。
    static constexpr int kMaxDriveRpm = 200;

    // 电机到轮子的传动比（轮子转速 / 电机转速）。
    static constexpr double kDriveGearRatio = 5.0 / 3.0;

    // 传感器
    static constexpr int kImuPort = 21;

    // ---- 自动赛 / 里程计。这三行必须上车量，量错了所有路径都按比例歪 ----------
    static constexpr double kWheelDiameterIn = 3.25;  // 轮子直径（英寸），磨过就重新量

    // 电机转 1 度，车走多少英寸：πD × 齿比 ÷ 360。
    // 轮子转一圈 = 360 ÷ 齿比 ≈ 216 电机度，和实测的「手转轮子 10 圈 = 2160 度」对得上。
    static constexpr double kInPerMotorDeg =
        3.14159265358979 * kWheelDiameterIn * kDriveGearRatio / 360.0;

    // 编码器方向。左电机 reverse = true，若自动赛里前进时 leftPosition() 读数为负，
    // 说明读数还在电机坐标系里，把它改成 -1.0。
    static constexpr double kLeftEncoderSign = 1.0;

    // 转向闭环（turnTo）
    static constexpr double kTurnKp = 2.0;         // rpm / 度
    static constexpr double kTurnMaxRpm = 120.0;   // 上限，太快会过冲
    static constexpr double kTurnMinRpm = 12.0;    // 下限，太小顶不动静摩擦
    static constexpr double kTurnTolDeg = 0.4;     // 到位判定（度）

    // 直线闭环（driveDistance）
    static constexpr double kDriveKp = 6.0;        // rpm / 英寸
    static constexpr double kDriveMaxRpm = 120.0;
    static constexpr double kDriveMinRpm = 15.0;
    static constexpr double kDriveTolIn = 0.4;     // 到位判定（英寸）
    static constexpr double kHeadingKp = 2.0;      // 直线纠偏 rpm / 度

    // 闭环公共参数
    static constexpr double kAccelRpmPerSec = 400.0;  // 加速斜率，防起步打滑
    static constexpr int kCtrlPeriodMs = 10;          // 闭环周期

    // 前进 / 后退曲线（左杆 Y）
    static constexpr int kPowerDeadband = 8;          // 死区：中位抖动直接吃掉
    static constexpr double kPowerMinOutput = 10.0;   // 刚出死区时的最小输出，用来顶过静摩擦
    static constexpr double kPowerCurveGain = 1.01;   // 曲线强度。1 = 线性，越大中心越"软"

    // 转向曲线（右杆 X）。和前进分开调：
    // 死区更大 —— 手抖不该让车头自己歪；增益大一点 —— 小角度转向能一点一点修。
    static constexpr int kTurnDeadband = 12;
    static constexpr double kTurnMinOutput = 14.0;    // 转向是左右轮反着走，静摩擦比直行大
    static constexpr double kTurnCurveGain = 1.03;
};
extern Chassis g_chassis;
extern pros::Controller g_master;

// 主控屏幕。构造顺序有讲究：它持有上面两个对象的引用，必须排在它们后面。
extern Display g_display;
