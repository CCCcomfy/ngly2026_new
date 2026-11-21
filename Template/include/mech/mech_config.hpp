#pragma once
// 机构唯一配置，改参数只改这里。带实测的必须上车量，其余按估值也能先跑。

#include <cstdint>

// 端口,具体后面实机再改
constexpr std::uint8_t PORT_SLIDE_A = 11;
constexpr std::uint8_t PORT_SLIDE_B = 12;
constexpr std::uint8_t PORT_ARM_A = 1;
constexpr std::uint8_t PORT_ARM_B = 18;
constexpr char PORT_CLAW_SOL = 'A';  //双作用缸 + 5/2 单电控阀，断电回默认位

// 同轴双电机必有一个反转，否则通电即互相顶死。
// 只接 A 确认方向，再接 B 给小行程，有异响就把对应项改 true。
constexpr bool SLIDE_B_REVERSED = true;
constexpr bool ARM_B_REVERSED = true;

// 整根滑轨轴的方向：true = 两个电机一起翻，让「正电压 = 上升」。
// 只翻手动电压是不够的——归零往下顶用的是 -3000，PID 和限位也都按「正=上升」写的。
constexpr bool SLIDE_DIR_REVERSED = true;

// 实测：从 0 推到 10 英寸，读电机角度变化 Δ，填 10.0 / Δ。
constexpr double SLIDE_IN_PER_DEG = 0.00972;

// 电机 5 圈 = 臂 3 圈。1.67:1 转速快、扭矩小，托举力只有 5:1 方案的三分之一，这个后面实测再改
constexpr double ARM_MOTOR_DEG_PER_ARM_DEG = 5.0 / 3.0;

// 软限位32，实测后把 MAX 收到留 5° 余量。
// ARM_MIN 必须小于 GROUND / TRAVEL 的角度，否则按键后停在半路。
constexpr double SLIDE_MIN_IN = 0.0;
constexpr double SLIDE_MAX_IN = 5.1;
constexpr double ARM_MIN_DEG = -70.00;
constexpr double ARM_MAX_DEG = 45.0;

constexpr double SLIDE_KP_MV_PER_IN = 900.0;
constexpr double SLIDE_KG_MV = 700.0;  // 托住负载的基础电压
constexpr double SLIDE_TOL_IN = 0.2;

// 理论起点：J=m*L^2=0.8*0.2^2=0.032 kg*m^2，水平重力矩 1.57 N*m。
// 以 KG=3500 能托住水平估算 Kt≈1.57/3500，折算 Jv≈1.24 mV*s^2/deg。
// 取 ωn≈8.5 rad/s、ζ≈1.0：KP≈90，KD≈21；这里 KD 取 25，略过阻尼。
constexpr double ARM_KP_MV_PER_DEG = 90.0;

// 积分只在稳态补静摩擦/电池差异；限幅 20 deg*s，防止顶限位时饱和。
constexpr double ARM_KI_MV_PER_DEG_SEC = 45.0;
constexpr double ARM_I_LIMIT_DEG_SEC = 20.0;

// 从 800 起，每次加 200，直到水平位不下滑。
// 到 4000 还托不住是减速比不够，再加只会烧电机。
constexpr double ARM_KG_MV = 3500.0;
constexpr double ARM_TOL_DEG = 1.5;

constexpr int SLIDE_MAX_MV = 10000;
constexpr int ARM_MAX_MV = 10000;
constexpr double SLIDE_SLEW_MV_PER_SEC = 40000.0;

// 1.67:1 能跑到 720°/s，不限速会在 0.13 秒内把件甩出去。
// 但限太狠也不行：手动满压 10000 切回保持 ~3000，中间差 7000 mV，
// 22 V/s 要爬 0.32 秒，臂就在这段空档里掉下去了。80 V/s 下只要 0.09 秒。
constexpr double ARM_SLEW_MV_PER_SEC = 80000.0;

// 位置环的微分：阻尼，不放大编码器噪声；理论临界值约 21，取 25。
constexpr double ARM_KD_MV_PER_DEG_PER_SEC = 25.0;

// 松手后速度降到这个值以下，才从「速度环刹车」切回「位置环保持」。
// 切太早：臂还带着速度，位置环一上来就是反向大电流，看起来像掉了一下再弹回来。
constexpr double ARM_STOP_DPS = 12.0;

// 上升别给满压；下降有自重，全压会砸到底。
constexpr int SLIDE_MANUAL_UP_MV = 7000;
constexpr int SLIDE_MANUAL_DOWN_MV = -2500;
// 手动电压。摇臂要叠加 ~3500 mV 的重力前馈才托得住，
// 手动只给 2500 时抬臂的净输出只剩 300 mV，顶不过静摩擦 —— 看起来就是「按了没反应」。
constexpr int ARM_MANUAL_MV = 8000;

// 手动改成「速度目标 + 速度 PI」：满杆 120 deg/s，前馈 8000/120≈66.7 mV/(deg/s)。
// 这样负角度不会一按就满压硬顶，速度环会把抖动压下去；顶住不动时积分再补力。
constexpr double ARM_MANUAL_MAX_DPS = 120.0;
constexpr double ARM_MANUAL_KP_MV_PER_DPS = 50.0;
constexpr double ARM_MANUAL_KI_MV_PER_DPS_SEC = 80.0;

// 通电到真正夹紧的时间。给少了会十几趟掉一趟件，而且不复现。
constexpr int CLAW_ACT_MS = 220;

constexpr double TEMP_LIMIT_C = 55.0;  // 超过降一半功率
constexpr int ACTION_TIMEOUT_MS = 3500;
constexpr int UPDATE_PERIOD_MS = 10;
