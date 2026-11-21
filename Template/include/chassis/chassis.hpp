#pragma once

#include <cstdint>

#include "main.h"
#include "chassis/drive_curve.hpp"
// chassis —— 底盘类
class Chassis {
public:
    // 构造底盘。端口从 Config 读
    Chassis();

    // 初始化：设置刹车方式、清零编码器、启动里程计任务。自动赛每次进入要重做。
    void initialize();

    // 按功率驱动：-127 ~ 127，超出限幅。开环电压，电池掉压后同样的值会变慢。
    void drivePower(int left, int right);

    // 按转速驱动：单位 rpm，超出 ±Config::kMaxDriveRpm 限幅。
    // 电机内部用 PID 稳速，电池电压和负载变化时速度基本不变。
    void driveVelocity(double left_rpm, double right_rpm);

    // 摇杆 → 曲线 → 转速（左右各一条杆，坦克式）。手动赛用，自动赛别用。
    void tank(int stick_left, int stick_right);

    // 街机驱动：左杆只管前进后退，右杆 X 只管转向（右推右转）。手动赛用，自动赛别用。
    void arcade(int power_stick, int turn_stick);

    // 停车：目标转速 0，按当前刹车模式停住（BRAKE 会主动抗住）
    void stop();

    // 校准 IMU（阻塞约 2 秒，期间车必须静止）
    void resetImu();

    // 把当前朝向记作 0 度，不重新校准，几毫秒完成。自动赛进场时用。
    void tareHeading();

    // ---- 自动赛接口 ----------------------------------------------------------
    // 位姿：x / y 单位英寸，theta 单位度（顺时针为正，范围 [0, 360)）
    struct Pose2D {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };

    // 当前位姿（里程计推算，会随漂移累积）
    Pose2D getPose() const;

    // 直接指定当前位姿。自动赛开头把车摆到起始点时调用，之后里程计从这里接着算。
    void setPose(double x, double y, double theta);

    // 原地转到 target_deg（IMU 闭环）。返回 false = 超时没到位。
    bool turnTo(double target_deg, int timeout_ms = 3000);

    // 直行 inches 英寸（编码器闭环 + 朝向纠偏），负值后退。返回 false = 超时没到位。
    bool driveDistance(double inches, int timeout_ms = 3000);

    // 里程计刷新。内部 10ms 任务调用，业务代码不用管。
    void updateOdometry();

    // 最近一次自动动作，给手柄调试屏用。
    // 动作名："DRV"（直行）/ "TURN"（转向）/ "-"（空闲）
    const char* autoAction() const { return m_action; }

    // 最近一次动作的实时误差：直行是英寸，转向是度。动作结束归 0。
    double autoError() const { return m_action_error; }

    // ---- 状态查询
    double getHeading();                  // 当前朝向 [0, 360)
    double leftPosition();                // 左侧编码器原始值（同侧多电机取平均）
    double rightPosition();               // 右侧编码器原始值
    double leftVelocity();                // 左侧实际转速（电机侧 rpm，同侧多电机取平均）
    double rightVelocity();               // 右侧实际转速
    double wheelRpm();                    // 车体实际转速，已按 kDriveGearRatio 折算到轮侧
    int maxTemperature();                 // 两侧最热的电机温度（摄氏度）

private:
    // 单侧行程，单位英寸（已按 kInPerMotorDeg 和编码器方向换算）
    double leftInches();
    double rightInches();
    double travelInches();

    pros::Motor_Group m_left_drive;   // 左侧两个电机
    pros::Motor_Group m_right_drive;  // 右侧两个电机

    DriveCurve m_power_curve;  // 前进 / 后退曲线（左杆 Y）
    DriveCurve m_turn_curve;   // 转向曲线（右杆 X）

    pros::Imu m_imu;  // 朝向传感器

    double m_pose_x = 0.0;      // 里程计 x（英寸）
    double m_pose_y = 0.0;      // 里程计 y（英寸）
    double m_pose_theta = 0.0;  // 里程计朝向（度）
    double m_last_left_in = 0.0;
    double m_last_right_in = 0.0;
    bool m_odom_ready = false;  // 第一次采样只做基准，不产生位移
    bool m_odom_started = false;

    const char* m_action = "-";   // 最近一次自动动作名（给调试屏）
    double m_action_error = 0.0;  // 该动作的实时误差
};
