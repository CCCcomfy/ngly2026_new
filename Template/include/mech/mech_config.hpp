#pragma once
// 机构唯一配置，改参数只改这里。带实测的必须上车量，其余按估值也能先跑。

#include <cstdint>

// 端口,具体后面实机再改
constexpr std::uint8_t PORT_SLIDE_A = 11;
constexpr std::uint8_t PORT_SLIDE_B = 12;
constexpr std::uint8_t PORT_ARM_A = 13;
constexpr std::uint8_t PORT_ARM_B = 14;
constexpr char PORT_CLAW_SOL = 'A';  //双作用缸 + 5/2 单电控阀，断电回默认位

// 同轴双电机必有一个反转，否则通电即互相顶死。
// 只接 A 确认方向，再接 B 给小行程，有异响就把对应项改 true。
constexpr bool SLIDE_B_REVERSED = true;
constexpr bool ARM_B_REVERSED = true;

// 实测：从 0 推到 10 英寸，读电机角度变化 Δ，填 10.0 / Δ。
constexpr double SLIDE_IN_PER_DEG = 0.00972;

// 电机 5 圈 = 臂 3 圈。1.67:1 转速快、扭矩小，托举力只有 5:1 方案的三分之一，这个后面实测再改
constexpr double ARM_MOTOR_DEG_PER_ARM_DEG = 5.0 / 3.0;

// 软限位32，实测后把 MAX 收到留 5° 余量。
// ARM_MIN 必须小于 GROUND / TRAVEL 的角度，否则按键后停在半路。
constexpr double SLIDE_MIN_IN = 0.0;
constexpr double SLIDE_MAX_IN = 2.0;
constexpr double ARM_MIN_DEG = -50.0;
constexpr double ARM_MAX_DEG = 60.0;

constexpr double SLIDE_KP_MV_PER_IN = 900.0;
constexpr double SLIDE_KG_MV = 700.0;  // 托住负载的基础电压
constexpr double SLIDE_TOL_IN = 0.2;

// 从 150 起调，每次加 20。1.67:1 下 1 度误差的角速度是 5:1 的三倍，沿用会振荡。
constexpr double ARM_KP_MV_PER_DEG = 180.0;

// 从 800 起，每次加 200，直到水平位不下滑。
// 到 4000 还托不住是减速比不够，再加只会烧电机。
constexpr double ARM_KG_MV = 2200.0;
constexpr double ARM_TOL_DEG = 1.5;

constexpr int SLIDE_MAX_MV = 10000;
constexpr int ARM_MAX_MV = 10000;
constexpr double SLIDE_SLEW_MV_PER_SEC = 40000.0;

// 1.67:1 能跑到 720°/s，不限速会在 0.13 秒内把件甩出去。
constexpr double ARM_SLEW_MV_PER_SEC = 22000.0;

// 上升别给满压；下降有自重，全压会砸到底。
constexpr int SLIDE_MANUAL_UP_MV = 7500;
constexpr int SLIDE_MANUAL_DOWN_MV = -4500;
constexpr int ARM_MANUAL_MV = 2500;

// 通电到真正夹紧的时间。给少了会十几趟掉一趟件，而且不复现。
constexpr int CLAW_ACT_MS = 220;

constexpr double TEMP_LIMIT_C = 55.0;  // 超过降一半功率
constexpr int ACTION_TIMEOUT_MS = 3500;
constexpr int UPDATE_PERIOD_MS = 10;
