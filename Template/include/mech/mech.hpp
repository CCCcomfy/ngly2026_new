#pragma once
// 机构对外接口：滑轨升降、摇臂转动、气动爪。不碰底盘和手柄。

#include <cstdint>

#include "mech/mech_config.hpp"

namespace slide {

void init();
void update();                     // 10ms 周期，由 mech::init() 起的任务驱动

void toInches(double inches);      // 阻塞到到位或超时
void toInchesAsync(double inches);
void nudge(double deltaIn);        // 在当前目标上微调
void manual(int mv);               // 手柄直通，传 0 = 松手刹车
void stop();
void hold();

bool isBusy();
bool isAtTarget();
double getInches();
double getTemperature();

void tare();
bool zeroByStall(int timeoutMs = 3000);   // 慢速下到底，堵转处定为 0

}  // namespace slide

namespace arm {

void init();
void update();

void toAngle(double deg);          // 阻塞到到位或超时
void toAngleAsync(double deg);
void nudge(double deltaDeg);
void manual(int mv);               // 传 0 = 松手转闭环托住，不是断电
void stop();
void hold();                       // 把目标设成当前角度继续托住（摇臂不能断电）

bool isBusy();
bool isAtTarget();
double getAngle();
double getTemperature();

void tare();
bool zeroByStall(int timeoutMs = 3000);

}  // namespace arm

namespace claw {

void init();

void grab();                       // 内部已等气缸行程，外面不要再 delay
void release();
void toggle();
bool isClosed();                   // 程序记账，不代表气缸真的到位

void grabNoWait();                 // 只发指令不等，给需要自己控时序的场合
void releaseNoWait();

}  // namespace claw

namespace mech {

struct Pose {
    double slideIn;
    double armDeg;
    bool clawClosed;
};

// 六组预设位姿。全是估值，必须手动摆到位后抄屏幕读数改。
// 角度为负 = 往车头方向下压，与 ARM_MIN/MAX 同一约定。
inline constexpr Pose GROUND{0.0, -12.0, false};     // 取件
inline constexpr Pose LOW_GOAL{15.0, 5.0, false};     // 低塔
inline constexpr Pose TALL_GOAL{25.0, -8.0, false};  // 高塔
inline constexpr Pose TRAVEL{0.0, -3.0, true};       // 行驶位
inline constexpr Pose CARRY{4.0, 15.0, true};        // 搬运
inline constexpr Pose STACK{10.0, 20.0, true};       // 叠放

void init();
void startTask();

// 上电归零，两轴各最多 3 秒，只在 initialize() 里调一次。
// 先滑轨后摇臂：滑轨降到底之后摇臂的转动空间最大。
void home();

bool goTo(const Pose& p, int timeoutMs = 2500);  // false = 超时或参数越界
void goToAsync(const Pose& p);                   // 只动两轴，爪子用不等版本

bool isBusy();
void emergencyStop();
void printStatus(int row);

// 机构单独调试，塞进 opcontrol 的 while 里，比赛前注释掉
//void testUpdate();

// 上层（屏幕等）只读的状态查询
inline bool isClampClosed() { return claw::isClosed(); }

}  // namespace mech
