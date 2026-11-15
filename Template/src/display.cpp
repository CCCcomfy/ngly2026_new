#include "display.hpp"

Display::Display(Chassis& chassis, Robot& robot, pros::Controller& master)
    : m_chassis(chassis), m_robot(robot), m_master(master) {
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

    // 第 5 行：夹爪状态
    pros::lcd::print(5, "CLAMP %s", m_robot.isClampClosed() ? "CLOSED" : "OPEN");

}
