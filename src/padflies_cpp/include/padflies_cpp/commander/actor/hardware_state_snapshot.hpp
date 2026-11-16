#pragma once

#include "rclcpp/time.hpp"
#include <cstdint>

namespace padflies_cpp {
struct HardwareStateSnapshot {
    bool can_fly = false;
    bool flying = false;
    bool tumbled = false;
    std::uint64_t sequence = 0;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
};

}  // namespace padflies_cpp
