#include "chassis.hpp"

#include <algorithm>
#include <cstdlib>

#include "robot_config.hpp"

namespace {

constexpr int kStickDeadband = 5;

// 单个电机功率的上下限
constexpr int kMaxPower = 127;

// 限幅：把任意整数限制在 -127 ~ 127
int clamp_power(int value) {
    return std::max(-kMaxPower, std::min(kMaxPower, value));
}

// 死区：绝对值小于阈值的输入一律当成 0
int apply_deadband(int value) {
    return (std::abs(value) < kStickDeadband) ? 0 : value;
}

}  // namespace

Chassis::Chassis()
    : m_left_drive({pros::Motor(Config::kLeftFrontPort, Config::kDriveGearset, false),
                    pros::Motor(Config::kLeftMidPort, Config::kDriveGearset, false),
                    pros::Motor(Config::kLeftBackPort, Config::kDriveGearset, false)}),
      m_right_drive({pros::Motor(Config::kRightFrontPort, Config::kDriveGearset, true),
                     pros::Motor(Config::kRightMidPort, Config::kDriveGearset, true),
                     pros::Motor(Config::kRightBackPort, Config::kDriveGearset, true)}),
      m_imu(Config::kImuPort) {}

void Chassis::initialize() {
    // 刹车方式
    m_left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    // 编码器清零
    // 自动赛每次进入都必须重做
    m_left_drive.tare_position();
    m_right_drive.tare_position();
}

void Chassis::drive(int left, int right) {
    m_left_drive.move(clamp_power(left));
    m_right_drive.move(clamp_power(right));
}

void Chassis::arcade(int power, int turn) {
    const int forward = apply_deadband(power);
    const int steer = apply_deadband(turn);

    int left = forward + steer;
    int right = forward - steer;

    const int max_magnitude = std::max(std::abs(left), std::abs(right));
    if (max_magnitude > kMaxPower) {
        left = left * kMaxPower / max_magnitude;
        right = right * kMaxPower / max_magnitude;
    }

    drive(left, right);
}

void Chassis::stop() {
    drive(0, 0);
}

void Chassis::resetImu() {
    // 阻塞式校准：这一行返回之前车必须完全静止。
    // 否则 IMU 会把「启动时的晃动」当成零位，之后所有转向都是歪的。
    m_imu.reset(true);
}

void Chassis::tareHeading() {
    m_imu.tare_heading();
}
