#pragma once

#include "drone_node/ctrl/algorithm/algorithm_base.hpp"

namespace drone_node::algorithm
{

// Transitional aliases for existing PID and FlightController code.
// These names refer only to mathematical algorithms.
// Physical controllers use drone_node::controller::ControllerBase.

template<int N>
using ControlVector = AlgorithmVector<N>;

template<int N>
using ControllerInput = AlgorithmInput<N>;

template<int N>
using ControllerOutput = AlgorithmOutput<N>;

template<int N>
using ControllerBase = AlgorithmBase<N>;

}  // namespace drone_node::algorithm