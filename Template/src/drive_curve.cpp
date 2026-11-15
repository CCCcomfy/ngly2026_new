#include "drive_curve.hpp"

#include <cstdlib>

namespace {
// 摇杆的满量程
constexpr int kMaxInput = 127;
}  // namespace

DriveCurve::DriveCurve(int deadband, double blend)
    : m_deadband(deadband), m_blend(blend) {
}

int DriveCurve::apply(int raw) const {
    // 死区设得比满量程还大，等于永远不动，避免下面除零
    if (m_deadband >= kMaxInput) {
        return 0;
    }

    // 1. 死区：中位附近的抖动直接吃掉。不设的话松手车会自己慢慢爬。
    const int magnitude = std::abs(raw);
    if (magnitude <= m_deadband) {
        return 0;
    }

    // 2. 重新归一化：死区边缘对应 0，摇杆推到底对应 1。
    //    这一步不能省 —— 如果只把死区内清零、外面照原样算，
    //    刚出死区时输出会从 0 直接跳到 deadband/127，车会"顿"一下。
    double x = static_cast<double>(magnitude - m_deadband) / static_cast<double>(kMaxInput - m_deadband);
    if (x > 1.0) {
        x = 1.0;
    }

    // 3. 三次曲线和线性曲线按比例混合。
    //    纯三次中心极平、微调很爽，但中段功率涨得太慢，手感发"绵"；
    //    掺一点线性把中段拉回来。blend 越大，中心越细腻、边缘越"闷"。
    const double curved = m_blend * x * x * x + (1.0 - m_blend) * x;

    // 4. 回到 -127 ~ 127。+0.5 是四舍五入，否则小数被截断，整体功率永远偏小一点
    const int output = static_cast<int>(curved * static_cast<double>(kMaxInput) + 0.5);
    return (raw < 0) ? -output : output;
}
