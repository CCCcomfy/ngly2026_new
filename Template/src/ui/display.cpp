#include "ui/display.hpp"
#include "chassis/chassis_config.hpp"
#include "input/input.hpp"

#include <cmath>

#include "mech/mech.hpp"

Display::Display(Chassis &chassis, pros::Controller &master)
    : m_chassis(chassis), m_master(master), m_peak_wheel_rpm(0.0)
{
}

Display g_display(g_chassis, g_master);

void Display::draw(const char *mode)
{
    // LLEMU
    // 当前模式（BOOT/READY/DISABLED/AUTO/OPCTRL）
    pros::lcd::print(0, "NGLY 2026      %s", mode);

    // 第 1 行：电池。
    pros::lcd::print(1, "BAT %5.1fV %3.0f%%  CTRL %s", pros::battery::get_voltage() / 1000.0,
                     pros::battery::get_capacity(), m_master.is_connected() ? "OK" : "--");

    // 第 2 行：朝向
    pros::lcd::print(2, "HEAD %7.1f deg", m_chassis.getHeading());

    // 第 3 行：左右轮编码器数值
    pros::lcd::print(3, "L %8.0f  R %8.0f", m_chassis.leftPosition(), m_chassis.rightPosition());

    // 第 4 行：同侧最热的电机温度
    pros::lcd::print(4, "TEMP %3d C", m_chassis.maxTemperature());

    // 第 5 行：机构状态
    if (!mech::taskRunning())
    {
        pros::lcd::print(5, "MECH TASK NOT RUNNING");
    }
    else if (!slide::sensorOk())
    {
        pros::lcd::print(5, "SLIDE ENCODER ERR");
    }
    else if (!arm::sensorOk())
    {
        pros::lcd::print(5, "ARM ENCODER ERR");
    }
    else
    {
        pros::lcd::print(5, "SL%5.1f AR%4.0f CLAMP %s", slide::getInches(), arm::getAngle(),
                         mech::isClampClosed() ? "C" : "O");
    }

    // 第 6 行：实际转速（电机侧 rpm，轮子转速）
    const double wheel = m_chassis.wheelRpm();
    if (std::fabs(wheel) > m_peak_wheel_rpm)
    {
        m_peak_wheel_rpm = std::fabs(wheel);
    }
    pros::lcd::print(6, "VEL L%4.0f R%4.0f  WHEEL %3.0f", m_chassis.leftVelocity(), m_chassis.rightVelocity(),
                     wheel);

    // 第 7 行：摇臂角度，实际电压，状态。
    char tail[8];
    std::snprintf(tail, sizeof(tail), "P%3.0f", m_peak_wheel_rpm);
    pros::lcd::print(7, "AR%6.1f CMD%5.0f %s", arm::getAngle(), arm::lastCommandMv(), tail);
}

void Display::resetPeak()
{
    m_peak_wheel_rpm = 0.0;
}
