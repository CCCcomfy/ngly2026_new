#include "ui/display.hpp"

#include <cmath>

#include "mech/mech.hpp"

Display::Display(Chassis& chassis, pros::Controller& master)
    : m_chassis(chassis), m_master(master), m_peak_wheel_rpm(0.0) {
}

void Display::draw(const char* mode) {
    // LLEMU 一共 8 行（0-7），每行约 40 个字符，超出的部分会被截断。
    // 第 0 行：身份 + 当前模式（BOOT / READY / DISABLED / AUTO / OPCTRL）
    pros::lcd::print(0, "NGLY 2026      %s", mode);

    // 第 1 行：电池。电压掉到 7V 以下主控就会重启
    pros::lcd::print(1, "BAT %5.1fV %3.0f%%  CTRL %s", pros::battery::get_voltage() / 1000.0,
                     pros::battery::get_capacity(), m_master.is_connected() ? "OK" : "--");

    // 第 2 行：朝向。调试转向闭环时最常看的一行
    pros::lcd::print(2, "HEAD %7.1f deg", m_chassis.getHeading());

    // 第 3 行：左右轮编码器原始值
    pros::lcd::print(3, "L %8.0f  R %8.0f", m_chassis.leftPosition(), m_chassis.rightPosition());

    // 第 4 行：同侧最热的电机温度。V5 电机到 55 度开始降功率，70 度停转
    pros::lcd::print(4, "TEMP %3d C", m_chassis.maxTemperature());

    // 第 5 行：机构状态。控制任务没起来、或者某个轴编码器读不到数时直接报错，
    // 别再显示一个看着正常的 CLAMP —— 这种「一切正常」的样子最容易误导排查。
    if (!mech::taskRunning()) {
        pros::lcd::print(5, "MECH TASK NOT RUNNING");
    } else if (!slide::sensorOk()) {
        pros::lcd::print(5, "SLIDE ENCODER ERR");
    } else if (!arm::sensorOk()) {
        pros::lcd::print(5, "ARM ENCODER ERR");
    } else {
        pros::lcd::print(5, "SL%5.1f AR%4.0f CLAMP %s", slide::getInches(), arm::getAngle(),
                         mech::isClampClosed() ? "C" : "O");
    }

    // 第 6 行：实际转速。左边两个数是电机侧 rpm，WHEEL 是折算到轮子上的转速。
    // 满杆应该是 200 / 200，WHEEL 是 333（60T 带 36T 增速）。
    const double wheel = m_chassis.wheelRpm();
    if (std::fabs(wheel) > m_peak_wheel_rpm) {
        m_peak_wheel_rpm = std::fabs(wheel);
    }
    pros::lcd::print(6, "VEL L%4.0f R%4.0f  WHEEL %3.0f", m_chassis.leftVelocity(), m_chassis.rightVelocity(),
                     wheel);

    // 第 7 行：摇臂角度 + 实际写出去的电压 + 上电最高轮速。
    // 按 R1/R2 时 CMD 应跳到 ~±5000（抬臂还会再叠 ~+2200 的重力前馈）：
    // CMD 有数而臂不动 = 电机/机械问题；CMD 一直是 0 = 按键没进程序或控制任务没跑。
    pros::lcd::print(7, "AR%6.1f CMD%5.0f PEAK%3.0f", arm::getAngle(), arm::lastCommandMv(), m_peak_wheel_rpm);
}

void Display::resetPeak() {
    m_peak_wheel_rpm = 0.0;
}
