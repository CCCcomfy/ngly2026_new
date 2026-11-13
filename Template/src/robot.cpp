#include "robot.hpp"

#include "robot_config.hpp"

namespace {
    constexpr int kLiftPower = 100;
}  // namespace

Robot::Robot()
    : m_lift(Config::kLiftPort, Config::kLiftGearset, false),
      m_clamp(Config::kClampPort, false) {}

void Robot::initialize() {
}

void Robot::liftUp() {
}

void Robot::liftDown() {
}

void Robot::liftStop() {
}

void Robot::clampOpen() {
}

void Robot::clampClose() {
}

void Robot::clampToggle() {
}
