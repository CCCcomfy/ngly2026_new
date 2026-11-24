#include "chassis/drive_curve.hpp"

#include <cmath>

namespace
{
    // 摇杆的满量程
    constexpr double kMaxInput = 127.0;
} // namespace

DriveCurve::DriveCurve(double deadband, double minOutput, double curveGain) : m_deadband(deadband < 0.0 ? 0.0 : deadband),
                                                                              m_min_output(minOutput < 0.0 ? 0.0 : (minOutput > kMaxInput ? kMaxInput : minOutput)),
                                                                              m_curve_gain(curveGain < 1.0 ? 1.0 : curveGain)
{
}

double DriveCurve::apply(double input) const
{
    // 防止除0
    if (m_deadband >= kMaxInput)
    {
        return 0.0;
    }

    // 死区
    if (std::fabs(input) <= m_deadband)
    {
        return 0.0;
    }

    const double sign = (input > 0.0) ? 1.0 : -1.0;

    // 出死区之后的行程
    const double g = std::fabs(input) - m_deadband;
    const double g_max = kMaxInput - m_deadband;

    // 指数映射
    const double i = std::pow(m_curve_gain, g - kMaxInput) * g * sign;
    const double i_max = std::pow(m_curve_gain, g_max - kMaxInput) * g_max;

    // 归一化到 127
    return (kMaxInput - m_min_output) / kMaxInput * i * kMaxInput / i_max + m_min_output * sign;
}
