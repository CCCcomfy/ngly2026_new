#pragma once

// DriveCurve —— 摇杆非线性映射
//   deadband  —— 死区
//   minOutput —— 最小输出。用来顶过电机静摩擦
//   curveGain —— 曲线强度
class DriveCurve {
public:
    DriveCurve(double deadband, double minOutput, double curveGain);
    // 输入输出都是 -127 ~ 127，正负号保持不变
    double apply(double input) const;

private:
    double m_deadband;    // 死区
    double m_min_output;  // 最小输出功率
    double m_curve_gain;  // 曲线增益
};
