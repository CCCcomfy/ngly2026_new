#include "chassis.hpp"

#include <algorithm>
#include <cstdlib>
#include <vector>
#include "drive_curve.hpp"
#include "robot_config.hpp"

namespace {
    // 单个电机功率的上下限
    constexpr int kMaxPower = 127;

    // 限幅：把任意整数限制在 -127 ~ 127
    int clamp_power(int value) {
        return std::max(-kMaxPower, std::min(kMaxPower, value));
}

// 同侧多电机的数值求平均。屏幕只需要一个数，取平均比分电机显示省地方。
double average(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

}  // namespace



Chassis::Chassis()
    : m_left_drive({pros::Motor(Config::kLeftFrontPort, Config::kDriveGearset, true),
                    pros::Motor(Config::kLeftBackPort, Config::kDriveGearset, true)}),
      m_right_drive({pros::Motor(Config::kRightFrontPort, Config::kDriveGearset, false),
                     pros::Motor(Config::kRightBackPort, Config::kDriveGearset, false)}),
      m_left_curve(Config::kStickDeadband, Config::kMinOutput, Config::kCurveGain),
      m_right_curve(Config::kStickDeadband, Config::kMinOutput, Config::kCurveGain),
      m_imu(Config::kImuPort) {}

void Chassis::initialize() {
    // 刹车方式
    m_left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    // 编码器清零
    m_left_drive.tare_position();
    m_right_drive.tare_position();
}

void Chassis::drive(int left, int right) {
    m_left_drive.move(clamp_power(left));
    m_right_drive.move(clamp_power(right));
}
void Chassis::tank(int stick_left, int stick_right) {
    drive(static_cast<int>(m_left_curve.apply(stick_left)),
          static_cast<int>(m_right_curve.apply(stick_right)));
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

double Chassis::getHeading() {
    return m_imu.get_heading();
}

double Chassis::leftPosition() {
    return average(m_left_drive.get_positions());
}

double Chassis::rightPosition() {
    return average(m_right_drive.get_positions());
}

int Chassis::maxTemperature() {
    double max_temp = 0.0;
    for (double temp : m_left_drive.get_temperatures()) {
        max_temp = std::max(max_temp, temp);
    }
    for (double temp : m_right_drive.get_temperatures()) {
        max_temp = std::max(max_temp, temp);
    }
    return static_cast<int>(max_temp);
}
