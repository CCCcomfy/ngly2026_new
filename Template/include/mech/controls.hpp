#pragma once
// 手动阶段只有这一个入口。键位是模块内部的事，上层不需要知道。
// 不阻塞、不循环，由 opcontrol 每 20ms 调一次。

namespace controls {

void task();

}
