#include "chassis/chassis_config.hpp"

Chassis g_chassis;

pros::Controller g_master(pros::E_CONTROLLER_MASTER);

// 必须放在 g_chassis / g_master 之后：构造函数要拿它们的引用
Display g_display(g_chassis, g_master);
