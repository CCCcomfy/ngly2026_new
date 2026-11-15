#pragma once

#include <cstdint>

#include "main.h"
#include "drive_curve.hpp"
// chassis —— 底盘类
class Chassis {
public:
    // 构造底盘。端口从 Config 读
    Chassis();

    // 初始化：设置刹车方式、清零编码器。自动赛每次进入要重做。
    void initialize();

    // 按功率驱动：-127 ~ 127，超出限幅。开环电压，电池掉压后同样的值会变慢。
    void drivePower(int left, int right);

    // 按转速驱动：单位 rpm，超出 ±Config::kMaxDriveRpm 限幅。
    // 电机内部用 PID 稳速，电池电压和负载变化时速度基本不变。
    void driveVelocity(double left_rpm, double right_rpm);

    // 摇杆 → 曲线 → 转速
    void tank(int stick_left, int stick_right);

    // 停车：目标转速 0，按当前刹车模式停住（BRAKE 会主动抗住）
    void stop();

    // 校准 IMU
    void resetImu();

    // 把当前朝向记作 0 度，不重新校准，几毫秒完成。自动赛进场时用。
    void tareHeading();

    // ---- 状态查询
    double getHeading();                  // 当前朝向 [0, 360)
    double leftPosition();                // 左侧编码器原始值（同侧多电机取平均）
    double rightPosition();               // 右侧编码器原始值
    double leftVelocity();                // 左侧实际转速（电机侧 rpm，同侧多电机取平均）
    double rightVelocity();               // 右侧实际转速
    double wheelRpm();                    // 车体实际转速，已按 kDriveGearRatio 折算到轮侧
    int maxTemperature();                 // 两侧最热的电机温度（摄氏度）

private:
    pros::Motor_Group m_left_drive;   // 左侧两个电机
    pros::Motor_Group m_right_drive;  // 右侧两个电机

    DriveCurve m_left_curve;   // 左杆曲线
    DriveCurve m_right_curve;  // 右杆曲线

    pros::Imu m_imu;  // 朝向传感器
};
