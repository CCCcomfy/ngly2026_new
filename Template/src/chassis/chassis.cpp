#include "chassis/chassis.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "chassis/drive_curve.hpp"
#include "chassis/chassis_config.hpp"

namespace
{
    constexpr int kMaxPower = 127;

    // 限幅：把任意整数限制在 -127 ~ 127
    int clamp_power(int value)
    {
        return std::max(-kMaxPower, std::min(kMaxPower, value));
    }

    // 限幅：把转速限制在最高转速
    double clamp_rpm(double value)
    {
        const double limit = 1.0 * Config::kMaxDriveRpm;
        return std::max(-limit, std::min(limit, value));
    }

    // 摇杆满行程 127 对应最高轮速，曲线输出换算成 rpm
    constexpr double kStickToRpm = 1.0 * Config::kMaxDriveRpm / kMaxPower;

    // 同侧多电机的数值求平均。屏幕只需要一个数，省显示空间。
    double average(const std::vector<double> &values)
    {
        if (values.empty())
        {
            return 0.0;
        }
        double sum = 0.0;
        for (double value : values)
        {
            sum += value;
        }
        return sum / values.size();
    }

    // 把角度差折到 [-180, 180]
    double wrap180(double deg)
    {
        while (deg > 180.0)
        {
            deg -= 360.0;
        }
        while (deg < -180.0)
        {
            deg += 360.0;
        }
        return deg;
    }

    // 里程计任务：10ms 采一次编码器，把位移累加进位姿
    void odometry_loop(void *params)
    {
        Chassis *chassis = static_cast<Chassis *>(params);
        while (true)
        {
            chassis->updateOdometry();
            pros::delay(Config::kCtrlPeriodMs);
        }
    }

} // namespace

Chassis::Chassis() : m_left_drive({pros::Motor(Config::kLeftFrontPort, Config::kDriveGearset, true),
                                   pros::Motor(Config::kLeftBackPort, Config::kDriveGearset, true)}),
                     m_right_drive({pros::Motor(Config::kRightFrontPort, Config::kDriveGearset, false),
                                    pros::Motor(Config::kRightBackPort, Config::kDriveGearset, false)}),
                     m_power_curve(Config::kPowerDeadband, Config::kPowerMinOutput, Config::kPowerCurveGain),
                     m_turn_curve(Config::kTurnDeadband, Config::kTurnMinOutput, Config::kTurnCurveGain),
                     m_imu(Config::kImuPort) {}

void Chassis::initialize()
{
    // 刹车方式
    m_left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    // 编码器清零
    m_left_drive.tare_position();
    m_right_drive.tare_position();

    // 里程计任务，只需做一次
    if (!m_odom_started)
    {
        m_odom_started = true;
        new pros::Task(odometry_loop, this, "odom");
    }

    // 位姿清零，编码器基准对齐到当前读数
    setPose(0.0, 0.0, 0.0);
}

void Chassis::drivePower(int left, int right)
{
    m_left_drive.move(clamp_power(left));
    m_right_drive.move(clamp_power(right));
}

void Chassis::driveVelocity(double left_rpm, double right_rpm)
{
    m_left_drive.move_velocity((std::int32_t)clamp_rpm(left_rpm));
    m_right_drive.move_velocity((std::int32_t)clamp_rpm(right_rpm));
}

void Chassis::tank(int stick_left, int stick_right)
{
    // 坦克式，左右两条杆控制左右电机，两侧都走前进曲线
    driveVelocity(m_power_curve.apply(stick_left) * kStickToRpm,
                  m_power_curve.apply(stick_right) * kStickToRpm);
}

void Chassis::arcade(int power_stick, int turn_stick)
{
    // 左杆 Y 走前进曲线，右杆 X 走转向曲线。
    const double power = m_power_curve.apply(power_stick);
    const double turn = m_turn_curve.apply(turn_stick);

    double left = power + turn;
    double right = power - turn;
    // 合成后可能超出 ±127，按最大幅值等比缩放
    const double peak = std::max(std::fabs(left), std::fabs(right));
    if (peak > kMaxPower)
    {
        const double scale = 1.0 * kMaxPower / peak;
        left *= scale;
        right *= scale;
    }

    driveVelocity(left * kStickToRpm, right * kStickToRpm);
}

void Chassis::stop()
{
    driveVelocity(0.0, 0.0);
}

// ---- 自动赛接口 --------------------------------------------------------------

Chassis::Pose2D Chassis::getPose() const
{
    Pose2D pose;
    pose.x = m_pose_x;
    pose.y = m_pose_y;
    pose.theta = m_pose_theta;
    return pose;
}

void Chassis::setPose(double x, double y, double theta)
{
    m_pose_x = x;
    m_pose_y = y;
    m_pose_theta = theta;

    // 用当前编码器重新做基准。防止设置位姿之前积累的位移算进来
    m_last_left_in = leftInches();
    m_last_right_in = rightInches();
    m_odom_ready = true;
}

// 顺时针为正
bool Chassis::turnTo(double target_deg, int timeout_ms)
{
    const std::uint32_t start_ms = pros::millis();

    while (pros::millis() - start_ms < timeout_ms)
    {
        const double err = wrap180(target_deg - getHeading());

        if (std::fabs(err) < Config::kTurnTolDeg)
        {
            stop();
            return true;
        }

        // P 控制：误差越大转得越快
        double out = Config::kTurnKp * err;
        out = std::max(-Config::kTurnMaxRpm, std::min(Config::kTurnMaxRpm, out));
        if (std::fabs(out) < Config::kTurnMinRpm)
        {
            out = (out >= 0.0 ? 1.0 : -1.0) * Config::kTurnMinRpm;
        }

        driveVelocity(out, -out);
        pros::delay(Config::kCtrlPeriodMs);
    }

    stop();
    return false;
}

bool Chassis::driveDistance(double inches, int timeout_ms)
{
    const double start_travel = travelInches();
    const double start_heading = getHeading();

    const std::uint32_t start_ms = pros::millis();
    std::uint32_t now_ms = start_ms;

    double min_err = std::fabs(inches); // 到目前为止最接近目标时差多少
    std::uint32_t stop_ms = 0;          // 误差开始不再变小的时刻
    double prev = 0.0;

    while (now_ms - start_ms < timeout_ms)
    {
        const double traveled = travelInches() - start_travel;
        const double err = inches - traveled;
        const double steer_err = wrap180(start_heading - getHeading());

        now_ms = pros::millis();

        if (std::fabs(err) < Config::kDriveTolIn)
        {
            stop();
            return true;
        }

        // 位置闭环：误差越大越快
        double out = Config::kDriveKp * err;
        out = std::max(-Config::kDriveMaxRpm, std::min(Config::kDriveMaxRpm, out));
        if (std::fabs(out) < Config::kDriveMinRpm)
        {
            out = (out >= 0.0 ? 1.0 : -1.0) * Config::kDriveMinRpm;
        }

        // 朝向纠偏
        double d = (steer_err - prev) / Config::kCtrlPeriodMs;
        double steer = Config::kHeadingKp * steer_err + Config::kHeadingKd * d;

        driveVelocity(out + steer, out - steer);
        prev = now_ms;

        // 车被卡住直接退出
        if (fabs(err) < min_err - 0.05)
        {
            min_err = fabs(err);
            stop_ms = 0;
        }
        else
        {
            if (stop_ms == 0)
                stop_ms = now_ms;
            if (now_ms - stop_ms > 700)
            {
                stop();
                return false;
            }
        }

        pros::delay(Config::kCtrlPeriodMs);
    }

    stop();
    return false;
}

void Chassis::updateOdometry()
{
    const double left = leftInches();
    const double right = rightInches();

    // 第一帧 基准读数
    if (!m_odom_ready)
    {
        m_last_left_in = left;
        m_last_right_in = right;
        m_odom_ready = true;
        return;
    }

    const double dl = left - m_last_left_in;
    const double dr = right - m_last_right_in;
    m_last_left_in = left;
    m_last_right_in = right;

    // 前进量取两侧平均，朝向直接用 IMU
    const double d = (dl + dr) / 2.0;
    const double theta_rad = getHeading() * 3.14159265358979 / 180.0;
    m_pose_x += d * std::cos(theta_rad);
    m_pose_y += d * std::sin(theta_rad);
    m_pose_theta = getHeading();
}

double Chassis::leftInches()
{
    return average(m_left_drive.get_positions()) * Config::kInPerMotorDeg * Config::kLeftEncoderSign;
}

double Chassis::rightInches()
{
    return average(m_right_drive.get_positions()) * Config::kInPerMotorDeg;
}

double Chassis::travelInches()
{
    return (leftInches() + rightInches()) / 2.0;
}

void Chassis::resetImu()
{
    m_imu.reset(true);
}

void Chassis::tareHeading()
{
    m_imu.tare_heading();
}

double Chassis::getHeading()
{
    return m_imu.get_heading();
}

double Chassis::leftPosition()
{
    return average(m_left_drive.get_positions());
}

double Chassis::rightPosition()
{
    return average(m_right_drive.get_positions());
}

double Chassis::leftVelocity()
{
    return average(m_left_drive.get_actual_velocities());
}

double Chassis::rightVelocity()
{
    return average(m_right_drive.get_actual_velocities());
}

double Chassis::wheelRpm()
{
    return (leftVelocity() + rightVelocity()) / 2.0 * Config::kDriveGearRatio;
}

int Chassis::maxTemperature()
{
    double max_temp = 0.0;
    for (double temp : m_left_drive.get_temperatures())
    {
        max_temp = std::max(max_temp, temp);
    }
    for (double temp : m_right_drive.get_temperatures())
    {
        max_temp = std::max(max_temp, temp);
    }
    return static_cast<int>(max_temp);
}
