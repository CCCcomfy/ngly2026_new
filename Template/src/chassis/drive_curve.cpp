#include "chassis/drive_curve.hpp"

#include <cmath>

namespace {
// 摇杆的满量程
constexpr double kMaxInput = 127.0;
}  // namespace

DriveCurve::DriveCurve(double deadband, double minOutput, double curveGain)
    : m_deadband(deadband < 0.0 ? 0.0 : deadband),
      m_min_output(minOutput < 0.0 ? 0.0 : (minOutput > kMaxInput ? kMaxInput : minOutput)),
      m_curve_gain(curveGain < 1.0 ? 1.0 : curveGain) {
}

double DriveCurve::apply(double input) const {
    // 死区比满量程还大，等于永远不动；顺便挡掉下面的除零
    if (m_deadband >= kMaxInput) {
        return 0.0;
    }

    // 1. 死区：中位附近的抖动直接吃掉
    if (std::fabs(input) <= m_deadband) {
        return 0.0;
    }

    const double sign = (input > 0.0) ? 1.0 : -1.0;

    // 2. 出死区之后的行程：刚出死区是 0，推到底最大
    const double g = std::fabs(input) - m_deadband;
    const double g_max = kMaxInput - m_deadband;

    // 3. 指数整形。curveGain = 1 时退化成直线；越大中心越"软"。
    const double i = std::pow(m_curve_gain, g - kMaxInput) * g * sign;
    const double i_max = std::pow(m_curve_gain, g_max - kMaxInput) * g_max;

    // 4. 先按满量程把 i 归一化到 127，再把 [minOutput, 127] 这段区间铺回去。
    //    满行程一定是 127，刚出死区一定是 minOutput。
    return (kMaxInput - m_min_output) / kMaxInput * i * kMaxInput / i_max + m_min_output * sign;
}
