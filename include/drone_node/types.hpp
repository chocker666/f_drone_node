
#pragma once

#include <Eigen/Core>

namespace drone_node
{

struct ReferenceState
{
    // Reference position in ENU, m.
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};

    // Reference velocity in ENU, m/s.
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};

    // Reference acceleration in ENU, m/s^2.
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};

    // Reference yaw and yaw rate, rad and rad/s.
    double yaw{0.0};
    double yaw_rate{0.0};
};

}  // namespace drone_node
