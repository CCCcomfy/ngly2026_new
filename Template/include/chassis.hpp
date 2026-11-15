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

    // 给左右两侧功率 -127 ~ 127，超出会限幅
    void drive(int left, int right);

    void tank(int stick_left,int stick_right);
    // 停车
    void stop();

    // 校准 IMU
    void resetImu();

    // 把当前朝向记作 0 度，不重新校准，几毫秒完成。自动赛进场时用。
    void tareHeading();

    // ---- 状态查询
    double getHeading();                  // 当前朝向 [0, 360)
    double leftPosition();                // 左侧编码器原始值（同侧多电机取平均）
    double rightPosition();               // 右侧编码器原始值
    int maxTemperature();                 // 两侧最热的电机温度（摄氏度）

private:
    pros::Motor_Group m_left_drive;   // 左侧两个电机
    pros::Motor_Group m_right_drive;  // 右侧两个电机

    DriveCurve m_left_curve;   // 左杆曲线
    DriveCurve m_right_curve;  // 右杆曲线

    pros::Imu m_imu;  // 朝向传感器
};
