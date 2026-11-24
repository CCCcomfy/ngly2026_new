#pragma once

#include <cstdint>

#include "main.h"
#include "chassis/drive_curve.hpp"
// chassis —— 底盘类
class Chassis {
public:
    Chassis();

    // 初始化：设置刹车方式、清零编码器、启动里程计任务。
    void initialize();

    // 按功率驱动：-127 ~ 127  开环电压
    void drivePower(int left, int right);

    // 按转速驱动  kMaxDriveRpm
    // 电机内部用 PID 稳速，电池电压和负载变化时速度基本不变。
    void driveVelocity(double left_rpm, double right_rpm);

    //坦克式驱动
    void tank(int stick_left, int stick_right);

    // 街机式驱动
    void arcade(int power_stick, int turn_stick);

    // 停车：目标转速 0，按当前刹车模式停车
    void stop();

    // 校准 IMU
    void resetImu();

    // 把当前朝向记作 0 度
    void tareHeading();

    // ------------------- 自动赛 ----------------------------
    // 位姿：x / y 单位英寸，theta 单位度（顺时针为正，范围 [0, 360)）
    struct Pose2D {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };

    // 当前位姿（里程计推算）
    Pose2D getPose() const;

    // 直接指定当前位姿。
    void setPose(double x, double y, double theta);

    // 原地转到目标度数（绝对）（IMU 闭环）。
    bool turnTo(double target_deg, int timeout_ms = 3000);

    // 直行 inches 英寸（编码器闭环 & 朝向修正），负值后退
    bool driveDistance(double inches, int timeout_ms = 3000);

    // 里程计刷新。内部 10ms 任务调用。
    void updateOdometry();

    // ---- 状态查询
    double getHeading();                  // 当前朝向 [0, 360)
    double leftPosition();                // 左侧编码器原始值（同侧多电机取平均）
    double rightPosition();               // 右侧编码器原始值
    double leftVelocity();                // 左侧实际转速（同侧多电机取平均）
    double rightVelocity();               // 右侧实际转速
    double wheelRpm();                    // 车体实际转速
    int maxTemperature();                 // 两侧最热的电机温度

private:
    // 单侧行程，单位英寸（按 kInPerMotorDeg 和编码器方向换算）
    double leftInches();
    double rightInches();
    double travelInches();

    pros::Motor_Group m_left_drive;   // 左侧电机组
    pros::Motor_Group m_right_drive;  // 右侧电机组

    DriveCurve m_power_curve;  // 前进曲线（左杆 Y）
    DriveCurve m_turn_curve;   // 转向曲线（右杆 X）

    pros::Imu m_imu;  // 朝向传感器

    double m_pose_x = 0.0;      // 里程计 x（英寸）
    double m_pose_y = 0.0;      // 里程计 y（英寸）
    double m_pose_theta = 0.0;  // 里程计朝向（度）
    double m_last_left_in = 0.0; 
    double m_last_right_in = 0.0; 
    bool m_odom_ready = false; 
    bool m_odom_started = false;
};
