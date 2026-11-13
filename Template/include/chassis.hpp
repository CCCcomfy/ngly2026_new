#pragma once

#include <cstdint>

#include "main.h"

// chassis —— 底盘类
class Chassis {
public:
    // 构造底盘。端口从 Config 读
    Chassis();

    // 初始化：设置刹车方式、清零编码器。自动赛每次进入要重做。
    void initialize();

    // 给左右两侧功率 -127 ~ 127，超出会限幅
    void drive(int left, int right);

    // 摇杆操作：power 前进分量，turn 转向分量 死区 合成
    void arcade(int power, int turn);

    // 停车
    void stop();

    // 校准 IMU
    void resetImu();

    // 把当前朝向记作 0 度，不重新校准，几毫秒完成。自动赛进场时用。
    void tareHeading();

private:
    pros::Motor_Group m_left_drive;   // 左侧三个电机
    pros::Motor_Group m_right_drive;  // 右侧三个电机
    pros::Imu m_imu;                  // 朝向传感器
};
