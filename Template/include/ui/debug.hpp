#pragma once

#include "main.h"

// 手柄调试屏。
//
// V5 手柄只有 3 行 × 15 列（PROS 限制：[0-2] 行 / [0-14] 列），
// 而且官方文档写明手柄显示还是 beta，刷新太快会丢字符，
// 所以这里固定 5Hz 刷新，并且内容没变就不重复写。
//
// 自动赛里这样用：
//   dbg::step("F24");                                   // 当前步骤名，最长 12 字符
//   if (!g_chassis.driveDistance(24, 2000)) dbg::fail(); // 没到位就标红
//
// 屏幕上三行分别是：
//   A F24            状态位 + 步骤名（A = 自动，! = 上一步失败，I = 空闲）
//   X -60 Y -12      里程计位姿（英寸）
//   DRV E  0.3       最近一次底盘动作和它的实时误差；空闲时显示 H 朝向 + B 电量
namespace dbg {
    // 起刷新任务并绑上手柄。在 initialize() 里调一次，重复调用无害。
    void start(pros::Controller& master);

    // 设置当前步骤名。传 nullptr 或 "" 表示空闲。会顺带清掉失败标记。
    void step(const char* name);

    // 标记上一步失败：状态位变成 "!". 下一次 step() 时清掉。
    void fail();
}  // namespace dbg
