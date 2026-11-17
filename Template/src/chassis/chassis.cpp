#include "chassis/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "chassis/drive_curve.hpp"
#include "core/robot_config.hpp"

namespace {
    // 单个电机功率的上下限
    constexpr int kMaxPower = 127;

    // 限幅：把任意整数限制在 -127 ~ 127
    int clamp_power(int value) {
        return std::max(-kMaxPower, std::min(kMaxPower, value));
    }

    // 限幅：把转速限制在 ±最高转速。驱动层再防一次，别指望电机固件悄悄兜底
    double clamp_rpm(double value) {
        const double limit = static_cast<double>(Config::kMaxDriveRpm);
        return std::max(-limit, std::min(limit, value));
    }

    // 摇杆满行程 127 对应最高轮速，曲线输出乘它换算成 rpm
    constexpr double kStickToRpm = static_cast<double>(Config::kMaxDriveRpm) / kMaxPower;

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

// 把角度差折到 [-180, 180]。不折的话 359° → 1° 会被当成「要转 358°」
double wrap180(double deg) {
    while (deg > 180.0) {
        deg -= 360.0;
    }
    while (deg < -180.0) {
        deg += 360.0;
    }
    return deg;
}

// 里程计任务：10ms 采一次编码器，把位移累加进位姿
void odometry_loop(void* params) {
    Chassis* chassis = static_cast<Chassis*>(params);
    while (true) {
        chassis->updateOdometry();
        pros::delay(Config::kCtrlPeriodMs);
    }
}

}  // namespace



Chassis::Chassis()
    : m_left_drive({pros::Motor(Config::kLeftFrontPort, Config::kDriveGearset, true),
                    pros::Motor(Config::kLeftBackPort, Config::kDriveGearset, true)}),
      m_right_drive({pros::Motor(Config::kRightFrontPort, Config::kDriveGearset, false),
                     pros::Motor(Config::kRightBackPort, Config::kDriveGearset, false)}),
      m_power_curve(Config::kPowerDeadband, Config::kPowerMinOutput, Config::kPowerCurveGain),
      m_turn_curve(Config::kTurnDeadband, Config::kTurnMinOutput, Config::kTurnCurveGain),
      m_imu(Config::kImuPort) {}

void Chassis::initialize() {
    // 刹车方式
    m_left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    m_right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    // 编码器清零
    m_left_drive.tare_position();
    m_right_drive.tare_position();

    // 里程计任务只起一次：自动赛每次进场都会调 initialize()，重复起任务会越攒越多
    if (!m_odom_started) {
        m_odom_started = true;
        new pros::Task(odometry_loop, this, "odom");
    }

    // 位姿清零，编码器基准对齐到当前读数
    setPose(0.0, 0.0, 0.0);
}

void Chassis::drivePower(int left, int right) {
    m_left_drive.move(clamp_power(left));
    m_right_drive.move(clamp_power(right));
}

void Chassis::driveVelocity(double left_rpm, double right_rpm) {
    m_left_drive.move_velocity(static_cast<std::int32_t>(clamp_rpm(left_rpm)));
    m_right_drive.move_velocity(static_cast<std::int32_t>(clamp_rpm(right_rpm)));
}

void Chassis::tank(int stick_left, int stick_right) {
    // 坦克式只有左右两条杆，没有单独的转向杆，两侧都走前进曲线
    driveVelocity(m_power_curve.apply(stick_left) * kStickToRpm,
                  m_power_curve.apply(stick_right) * kStickToRpm);
}

void Chassis::arcade(int power_stick, int turn_stick) {
    // 两条曲线各管一件事：左杆 Y 走前进曲线，右杆 X 走转向曲线。
    // 先各自过曲线再合成，前进和转向的手感就能分开调。
    const double power = m_power_curve.apply(power_stick);
    const double turn = m_turn_curve.apply(turn_stick);

    double left = power + turn;
    double right = power - turn;

    // 合成后可能超出 ±127，按最大幅值等比缩放。
    // 只砍超限的那一边会让车原地急转：前进 100 + 转向 100 时左轮 200 砍成 127、右轮 0。
    const double peak = std::max(std::fabs(left), std::fabs(right));
    if (peak > kMaxPower) {
        const double scale = static_cast<double>(kMaxPower) / peak;
        left *= scale;
        right *= scale;
    }

    driveVelocity(left * kStickToRpm, right * kStickToRpm);
}

void Chassis::stop() {
    driveVelocity(0.0, 0.0);
}

// ---- 自动赛接口 --------------------------------------------------------------

Chassis::Pose2D Chassis::getPose() const {
    Pose2D pose;
    pose.x = m_pose_x;
    pose.y = m_pose_y;
    pose.theta = m_pose_theta;
    return pose;
}

void Chassis::setPose(double x, double y, double theta) {
    m_pose_x = x;
    m_pose_y = y;
    m_pose_theta = theta;

    // 用当前编码器重新做基准。否则下一帧会把「设置位姿之前」积累的位移算进来，
    // 表现就是 setPose 之后车还没动，x 却自己跳了一段。
    m_last_left_in = leftInches();
    m_last_right_in = rightInches();
    m_odom_ready = true;
}

bool Chassis::turnTo(double target_deg, int timeout_ms) {
    const std::uint32_t start_ms = pros::millis();

    while (pros::millis() - start_ms < static_cast<std::uint32_t>(timeout_ms)) {
        const double err = wrap180(target_deg - getHeading());

        if (std::fabs(err) < Config::kTurnTolDeg) {
            stop();
            return true;
        }

        // P 控制：误差越大转得越快，同时限幅 + 下限，避免离目标很远时慢慢蹭
        double out = Config::kTurnKp * err;
        out = std::max(-Config::kTurnMaxRpm, std::min(Config::kTurnMaxRpm, out));
        if (std::fabs(out) < Config::kTurnMinRpm) {
            out = (out >= 0.0 ? 1.0 : -1.0) * Config::kTurnMinRpm;
        }

        // heading 顺时针为正：顺时针 = 左轮前进 + 右轮后退
        driveVelocity(out, -out);
        pros::delay(Config::kCtrlPeriodMs);
    }

    stop();
    return false;
}

bool Chassis::driveDistance(double inches, int timeout_ms) {
    const double start_travel = travelInches();
    const double start_heading = getHeading();

    double prev_out = 0.0;
    std::uint32_t prev_ms = pros::millis();
    const std::uint32_t start_ms = prev_ms;

    while (pros::millis() - start_ms < static_cast<std::uint32_t>(timeout_ms)) {
        const double traveled = travelInches() - start_travel;
        const double err = inches - traveled;

        if (std::fabs(err) < Config::kDriveTolIn) {
            stop();
            return true;
        }

        // 位置环：误差越大越快，超时前一直朝目标爬
        double out = Config::kDriveKp * err;
        out = std::max(-Config::kDriveMaxRpm, std::min(Config::kDriveMaxRpm, out));
        if (std::fabs(out) < Config::kDriveMinRpm) {
            out = (out >= 0.0 ? 1.0 : -1.0) * Config::kDriveMinRpm;
        }

        // 朝向纠偏：车头偏离起始朝向就往回打，否则两圈下来就是斜着走
        const double steer = Config::kHeadingKp * wrap180(start_heading - getHeading());

        // 斜率限制：一步从 0 跳到 120rpm 会让轮子空转，编码器和实际位移就对不上了
        const std::uint32_t now = pros::millis();
        const double dt_s = static_cast<double>(now - prev_ms) / 1000.0;
        const double max_delta = Config::kAccelRpmPerSec * dt_s;
        out = prev_out + std::max(-max_delta, std::min(max_delta, out - prev_out));
        prev_out = out;
        prev_ms = now;

        driveVelocity(out + steer, out - steer);
        pros::delay(Config::kCtrlPeriodMs);
    }

    stop();
    return false;
}

void Chassis::updateOdometry() {
    const double left = leftInches();
    const double right = rightInches();

    // 第一帧只做基准：这时还分不清「原始读数」和「上一帧读数」
    if (!m_odom_ready) {
        m_last_left_in = left;
        m_last_right_in = right;
        m_odom_ready = true;
        return;
    }

    const double dl = left - m_last_left_in;
    const double dr = right - m_last_right_in;
    m_last_left_in = left;
    m_last_right_in = right;

    // 前进量取两侧平均；朝向直接用 IMU，比编码器差分稳得多
    const double d = (dl + dr) / 2.0;
    const double theta_rad = getHeading() * 3.14159265358979 / 180.0;
    m_pose_x += d * std::cos(theta_rad);
    m_pose_y += d * std::sin(theta_rad);
    m_pose_theta = getHeading();
}

double Chassis::leftInches() {
    return average(m_left_drive.get_positions()) * Config::kInPerMotorDeg * Config::kLeftEncoderSign;
}

double Chassis::rightInches() {
    return average(m_right_drive.get_positions()) * Config::kInPerMotorDeg;
}

double Chassis::travelInches() {
    return (leftInches() + rightInches()) / 2.0;
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

double Chassis::leftVelocity() {
    // get_actual_velocities 返回电机实测转速（rpm），同侧两个电机取平均
    return average(m_left_drive.get_actual_velocities());
}

double Chassis::rightVelocity() {
    return average(m_right_drive.get_actual_velocities());
}

double Chassis::wheelRpm() {
    // 车体转速 = 左右平均（差速转向时左右不等，这里只作参考值）再按传动比折算到轮侧
    return (leftVelocity() + rightVelocity()) / 2.0 * Config::kDriveGearRatio;
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
