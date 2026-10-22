#pragma once

#include <vector>

#include "drone_node/ctrl/trajectory_sampler/trajectory_sampler_base.hpp"
#include "drone_node/msg/list_trajectory.hpp"

namespace drone_node::trajectory_sampler
{

class ListSampler final : public TrajectorySamplerBase
{
public:
    explicit ListSampler(
        const drone_node::msg::ListTrajectory& trajectory);

    ReferenceState sample(double t) const override;

    double duration() const override;

private:
    struct Profile
    {
        double distance{0.0};
        double acceleration{0.0};
        double ramp{0.0};
        double cruise{0.0};
        double peak{0.0};
        double duration{0.0};
    };

    struct Motion
    {
        double distance{0.0};
        double velocity{0.0};
        double acceleration{0.0};
    };

    struct LinearSegment
    {
        double begin{0.0};
        Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
        Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
        Profile profile{};
    };

    struct YawSegment
    {
        double begin{0.0};
        double origin{0.0};
        double direction{1.0};
        Profile profile{};
    };

    static Profile plan(
        double distance,
        double max_speed,
        double max_acceleration);

    static Motion evaluate(const Profile& profile, double t);

    static double yaw_delta(
        double current,
        double target,
        uint8_t direction);

    std::vector<LinearSegment> linear_segments_;
    std::vector<YawSegment> yaw_segments_;

    Eigen::Vector3d final_position_{Eigen::Vector3d::Zero()};
    double final_yaw_{0.0};

    double linear_duration_{0.0};
    double yaw_duration_{0.0};
};

}  // namespace drone_node::trajectory_sampler