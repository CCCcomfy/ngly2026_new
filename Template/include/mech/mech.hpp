#pragma once

#include <cstdint>

#include "mech/mech_config.hpp"

namespace slide {

void init();
void update();                    

void toInches(double inches);      // 阻塞
void toInchesAsync(double inches);
void nudge(double deltaIn);        // 在当前目标上微调
void manual(int mv);               // 手柄直通，传0松手刹车，设置电压
void stop();
void hold();

bool isBusy();
bool isAtTarget();
double getInches();
double getTemperature();

bool sensorOk();   // 编码器读得到有效数
bool isZeroed();   // 已经归零过

void tare();
bool zeroByStall(int timeoutMs = 3000);   // 慢速下到底，自动归零

}  // namespace slide

namespace arm {

void init();
void update();

void toAngle(double deg);          // 阻塞到到位或超时
void toAngleAsync(double deg);
void nudge(double deltaDeg);
void manual(int mv);               // 传0松手转闭环托住
void stop();                       // 停手动并原地托住
void hold();                       // 把目标设成当前角度继续托住
bool isBusy();
bool isAtTarget();
double getAngle();
double getTemperature();

bool sensorOk();   
bool isZeroed();  
double lastCommandMv();   

void tare();
bool zeroByStall(int timeoutMs = 3000);

}  // namespace arm

namespace claw {

void init();

void grab();                      
void release();
void toggle();
bool isClosed();                  

void grabNoWait();               
void releaseNoWait();
void toggleNoWait();               
bool isSettled();                 
}  // namespace claw

namespace mech {

struct Pose {
    double slideIn;
    double armDeg;
    bool clawClosed;
};

// 六组预设位姿。写自动赛的注意，这全是我猜的，参数没调
inline constexpr Pose GROUND{0.0, -60.0, false};     // 取件
inline constexpr Pose LOW_GOAL{1.0, -30.0, false};     // 低塔
inline constexpr Pose TALL_GOAL{5.0, 30.0, false};  // 高塔
inline constexpr Pose TRAVEL{0.0, -60.0, true};       // 行驶位
inline constexpr Pose CARRY{2.0, 15.0, true};        // 搬运
inline constexpr Pose STACK{4.0, 20.0, true};       // 叠放

void init();
void startTask();

bool taskRunning();   // 10ms 控制任务是否已经起来

// 上电归零
void home();

bool goTo(const Pose& p, int timeoutMs = 2500);  
bool goTo(double degree, int timeoutMs = 2500);
void goToAsync(const Pose& p);                   

bool isBusy();
void emergencyStop();
void printStatus(int row);


inline bool isClampClosed() { return claw::isClosed(); }

}  // namespace mech
