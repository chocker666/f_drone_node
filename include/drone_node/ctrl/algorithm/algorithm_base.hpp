#pragma once

#include <Eigen/Core>

namespace drone_node::algorithm
{

template<int N>
using AlgorithmVector = Eigen::Matrix<double, N, 1>;

// Pure mathematical input.
// Physical meaning and units are assigned by the calling controller.
template<int N>
struct AlgorithmInput
{
    AlgorithmVector<N> error{AlgorithmVector<N>::Zero()};
    AlgorithmVector<N> error_rate{AlgorithmVector<N>::Zero()};

    double dt{0.02};  // s
};

template<int N>
struct AlgorithmOutput
{
    AlgorithmVector<N> command{AlgorithmVector<N>::Zero()};
};

// Parameters belong to each concrete algorithm.
// There is no shared PID/LQR/MPC parameter structure.
template<int N>
class AlgorithmBase
{
public:
    static_assert(N >= 1, "Algorithm dimension must be positive");

    virtual ~AlgorithmBase() = default;

    virtual void reset() = 0;

    virtual AlgorithmOutput<N> update(
        const AlgorithmInput<N>& input) = 0;
};

}  // namespace drone_node::algorithm