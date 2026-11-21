#pragma once

#include "main.h"

// 手柄调试屏。
//
// V5 手柄只有 3 行 × 15 列（PROS 限制：[0-2] 行 / [0-14] 列），
// 而且官方文档写明手柄显示还是 beta，刷新太快会丢字符，
// 所以这里固定 5Hz 刷新，并且内容没变就不重复写。
//
// 自动赛里这样用：
//   dbg::step("F24");   // 当前步骤名，最长 12 字符
//
// 屏幕上三行分别是：
//   A F24            状态位 + 步骤名
//   X -60 Y -12      里程计位姿（英寸）
//   DRV E  0.3       最近一次底盘动作和它的实时误差；空闲时显示 H 朝向 + B 电量
//
// 状态位含义：
//   >  正在执行底盘动作
//   A  上一步动作到位（进了容差）
//   !  上一步动作超时（跑满 timeout 还没进容差，后面的路径基本都会歪）
//   I  还没有开始任何动作
namespace dbg {
    // 起刷新任务并绑上手柄。在 initialize() 里调一次，重复调用无害。
    void start(pros::Controller& master);

    // 设置当前步骤名。传 nullptr 或 "" 表示空闲。
    void step(const char* name);
}  // namespace dbg
