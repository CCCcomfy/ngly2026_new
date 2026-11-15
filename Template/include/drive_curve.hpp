#pragma once

// DriveCurve —— 摇杆非线性映射
//
// 把 get_analog() 给出来的 -127 ~ 127 原始值，换成同样范围、但手感完全不同的输出：
// 中心区细腻（推一点走一点），边缘区照样能给满功率。
//
// 直接拿摇杆值当功率的问题是：V5 摇杆中段很"陡"，轻轻一碰功率就跳到 40、50，
// 走直线、贴墙、微调位置全都没法做。曲线把中段压平、把边缘拉满，两个需求一次解决。
class DriveCurve {
public:
    // deadband：死区。|输入| 小于它直接输出 0（摇杆中位抖动通常在 3 ~ 8）
    // blend：三次项占比，0 ~ 1。0 = 纯线性，1 = 纯三次（中心最细腻）
    DriveCurve(int deadband, double blend);

    // 输入输出都是 -127 ~ 127，正负号保持不变
    int apply(int raw) const;

private:
    int m_deadband;
    double m_blend;
};
