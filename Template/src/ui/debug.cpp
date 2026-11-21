#include "ui/debug.hpp"

#include <cstdio>
#include <cstring>

#include "chassis/chassis.hpp"
#include "chassis/chassis_config.hpp"

namespace {

pros::Controller* s_master = nullptr;
char s_step[13] = "";       // 当前步骤名，最长 12 个字符
bool s_failed = false;      // 最近一次动作是否失败
char s_printed[3][16] = {}; // 已经写到屏幕上的三行内容

// 写一行：先补空格到 15 列（不补的话短字符串盖不住上一帧的长字符串），
// 再和上一帧比较，没变化就不写 —— 手柄屏幕每次写入都是串口包，
// 每帧都写会让显示明显滞后，竞技场上看到的就是「慢半拍」。
void putLine(int line, const char* text) {
    if (s_master == nullptr) {
        return;
    }

    char padded[16];
    std::snprintf(padded, sizeof(padded), "%-15s", text);

    if (std::strcmp(padded, s_printed[line]) == 0) {
        return;
    }

    std::memcpy(s_printed[line], padded, sizeof(padded));
    s_master->set_text(static_cast<std::uint8_t>(line), 0, padded);
}

void refresh() {
    if (s_master == nullptr) {
        return;
    }

    const Chassis::Pose2D pose = g_chassis.getPose();
    char buf[16];

    // 第 0 行：状态位 + 步骤名。I = 还没开始跑步骤，A = 正在跑，! = 上一步失败
    if (s_step[0] == '\0') {
        std::snprintf(buf, sizeof(buf), "%c IDLE", s_failed ? '!' : 'I');
    } else {
        std::snprintf(buf, sizeof(buf), "%c %s", s_failed ? '!' : 'A', s_step);
    }
    putLine(0, buf);

    // 第 1 行：位姿。取整就够判断「是不是差了一大截」，小数反而看不清
    std::snprintf(buf, sizeof(buf), "X%4.0f Y%4.0f", pose.x, pose.y);
    putLine(1, buf);

    // 第 2 行：有动作在跑就显示误差（调 P 增益主要看这个残留值），
    // 空闲时显示朝向和电量 —— 电量低于 20% 时自动赛一致性会明显变差
    const char* action = g_chassis.autoAction();
    if (std::strcmp(action, "-") != 0) {
        std::snprintf(buf, sizeof(buf), "%s E%5.1f", action, g_chassis.autoError());
    } else {
        std::snprintf(buf, sizeof(buf), "H%3.0f B%3.0f%%", g_chassis.getHeading(),
                      static_cast<double>(pros::battery::get_capacity()));
    }
    putLine(2, buf);
}

void loop(void*) {
    while (true) {
        refresh();
        pros::delay(200);
    }
}

}  // namespace

void dbg::start(pros::Controller& master) {
    if (s_master != nullptr) {
        return;
    }

    s_master = &master;
    s_master->clear_line(0);
    s_master->clear_line(1);
    s_master->clear_line(2);

    new pros::Task(loop, nullptr, "dbc");
}

void dbg::step(const char* name) {
    std::snprintf(s_step, sizeof(s_step), "%s", name == nullptr ? "" : name);
    s_failed = false;
}

void dbg::fail() {
    s_failed = true;
}
