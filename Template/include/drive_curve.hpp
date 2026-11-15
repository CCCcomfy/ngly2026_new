#pragma once

// DriveCurve —— 摇杆非线性映射
//   deadband  —— 死区，中位附近的抖动直接吃掉
//   minOutput —— 最小输出。刚出死区时至少给这么多，用来顶过电机静摩擦
//   curveGain —— 曲线强度。1 = 线性，越大中心段被压得越低
class DriveCurve {
public:
    // 构造时自动修正非法参数：负数归零、minOutput 不超过 127、curveGain 不小于 1
    DriveCurve(double deadband, double minOutput, double curveGain);

    // 输入输出都是 -127 ~ 127，正负号保持不变
    double apply(double input) const;

private:
    double m_deadband;    // 死区
    double m_min_output;  // 最小输出功率
    double m_curve_gain;  // 曲线增益
};
