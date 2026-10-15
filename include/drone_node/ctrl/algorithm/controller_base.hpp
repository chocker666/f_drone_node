
#pragma once

#include <Eigen/Core>

namespace drone_node::algorithm
{

template<int N>
using ControlVector = Eigen::Matrix<double, N, 1>;

template<int N>
struct ControllerInput
{
    ControlVector<N> error{
        ControlVector<N>::Zero()};

    ControlVector<N> error_rate{
        ControlVector<N>::Zero()};

    double dt{0.02};
};

template<int N>
struct ControllerOutput
{
    ControlVector<N> command{
        ControlVector<N>::Zero()};
};

template<int N>
class ControllerBase
{
public:
    static_assert(N >= 1, "Control dimension must be positive");

    virtual ~ControllerBase() = default;

    virtual void reset() = 0;

    virtual ControllerOutput<N> update(
        const ControllerInput<N>& input) = 0;
};

}  // namespace drone_node::algorithm
