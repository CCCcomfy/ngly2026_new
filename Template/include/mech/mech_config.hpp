#pragma once


#include <cstdint>


constexpr std::uint8_t PORT_SLIDE_A = 11;
constexpr std::uint8_t PORT_SLIDE_B = 12;
constexpr std::uint8_t PORT_ARM_A = 1;
constexpr std::uint8_t PORT_ARM_B = 18;
constexpr char PORT_CLAW_SOL = 'A';

constexpr bool SLIDE_B_REVERSED = true;
constexpr bool ARM_B_REVERSED = true;
//整个方向往上
constexpr bool SLIDE_DIR_REVERSED = true;


constexpr double SLIDE_IN_PER_DEG = 0.00972;

// 电机 5 圈 = 臂 3 圈
constexpr double ARM_MOTOR_DEG_PER_ARM_DEG = 5.0 / 3.0;


constexpr double SLIDE_MIN_IN = 0.0;
constexpr double SLIDE_MAX_IN = 5.1;
constexpr double ARM_MIN_DEG = -80.00;
constexpr double ARM_MAX_DEG = 45.0;
// 取消软件角度限位
constexpr bool ARM_SOFT_LIMIT_ENABLED = false;
// init() 用这个值建立角度零点。
constexpr double ARM_BOOT_DEG = -80.0;

constexpr double SLIDE_KP_MV_PER_IN = 900.0;
constexpr double SLIDE_KG_MV = 700.0;  // 托住负载的基础电压
constexpr double SLIDE_TOL_IN = 0.2;

//水平重力矩 大约1.57 N*m。
// 不用 cos 前馈后 P 取 180，水平托举电压由积分和到位偏置建立；KD 取 25。
constexpr double ARM_KP_MV_PER_DEG = 180.0;

//积分项
constexpr double ARM_KI_MV_PER_DEG_SEC = 120.0;
constexpr double ARM_I_LIMIT_DEG_SEC = 30.0;


constexpr double ARM_KG_MV = 3500.0;
constexpr double ARM_TOL_DEG = 1.5;

constexpr int SLIDE_MAX_MV = 10000;
constexpr int ARM_MAX_MV = 10000;
constexpr double SLIDE_SLEW_MV_PER_SEC = 40000.0;

//每秒变化毫伏
constexpr double ARM_SLEW_MV_PER_SEC = 80000.0;

// 位置环的微分
constexpr double ARM_KD_MV_PER_DEG_PER_SEC = 25.0;

// 松手后速度降到这个值以下，才从速度环刹车切回位置环保持。

constexpr double ARM_STOP_DPS = 12.0;


constexpr int SLIDE_MANUAL_UP_MV = 8000;
constexpr int SLIDE_MANUAL_DOWN_MV = -3000;

constexpr int ARM_MANUAL_MV = 10000;

// 手动改成速度目标 + 速度 PI
// 这样负角度不会一按就满压硬顶，速度环会把抖动压下去；顶住不动时积分再补力。
constexpr double ARM_MANUAL_MAX_DPS = 120.0;
constexpr double ARM_MANUAL_KP_MV_PER_DPS = 50.0;
constexpr double ARM_MANUAL_KI_MV_PER_DPS_SEC = 80.0;

// 从负角度一直到20度都给满压，越过20度后再交回速度环。
constexpr double ARM_MANUAL_LIFT_MAX_MV = 12000.0;
constexpr double ARM_MANUAL_LIFT_BOOST_DEG = 20.0;

// 通电到真正夹紧的时间。
constexpr int CLAW_ACT_MS = 220;

constexpr double TEMP_LIMIT_C = 55.0;  // 超过降一半功率
constexpr int ACTION_TIMEOUT_MS = 3500;
constexpr int UPDATE_PERIOD_MS = 10;
