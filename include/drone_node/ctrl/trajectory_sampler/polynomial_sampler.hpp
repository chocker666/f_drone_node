
#pragma once

#include <cstddef>
#include <vector>

#include "drone_node/types.hpp"
#include "drone_node/msg/control_trajectory.hpp"

namespace drone_node::trajectory_sampler
{

class PolynomialSampler final
{
public:
    explicit PolynomialSampler(
        const drone_node::msg::ControlTrajectory& trajectory);

    ReferenceState sample(double t) const;

    ReferenceSample sample_with_validity(double t) const;

    double duration() const;

private:
    struct Segment
    {
        double start_time{0.0};
        double duration{0.0};

        std::vector<double> coeff_x;
        std::vector<double> coeff_y;
        std::vector<double> coeff_z;
        std::vector<double> coeff_yaw;
    };

    static double evaluate(
        const std::vector<double>& coefficients,
        double t,
        unsigned int derivative_order);

    static void validate_boundary(
        const Segment& left,
        const Segment& right);

    std::vector<Segment> segments_;
    double duration_{0.0};
};

}  // namespace drone_node::trajectory_sampler
