#pragma once

#include "main.h"

// robot —— 上层机构类
// 抬升、夹爪
class Robot {
public:
    Robot();

    // 上电初始化：设置抬升电机的刹车方式、夹爪回到初始状态
    void initialize();

    // 抬升
    void liftUp();    // 上升
    void liftDown();  // 下降
    void liftStop();  // 停住

    // 夹爪（气动，只有开 / 关）
    void clampOpen();
    void clampClose();
    void clampToggle();  // 在开 / 关之间切换

private:
    pros::Motor m_lift;
    pros::ADIDigitalOut m_clamp;

    bool m_clamp_closed = true;
};
