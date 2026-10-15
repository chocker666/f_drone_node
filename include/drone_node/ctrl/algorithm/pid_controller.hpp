
#pragma once

#include "drone_node/ctrl/algorithm/controller_base.hpp"

namespace drone_node::algorithm
{

enum class PIDLimitMode
{
    NORM,
    PER_AXIS
};

struct PIDParameters
{
    double kp{0.8};
    double ki{0.0};
    double kd{0.0};

    double max_output{1.0};
    double max_output_rate{1.0};
    double integral_limit{1.0};

    PIDLimitMode limit_mode{PIDLimitMode::NORM};
};

template<int N>
class PIDController final : public ControllerBase<N>
{
public:
    using Vector = ControlVector<N>;

    explicit PIDController(
        const PIDParameters& parameters = PIDParameters{});

    void set_parameters(
        const PIDParameters& parameters);

    const PIDParameters& parameters() const noexcept;

    void reset() override;

    ControllerOutput<N> update(
        const ControllerInput<N>& input) override;

private:
    Vector limit_vector(
        const Vector& value,
        double limit) const;

    PIDParameters parameters_{};

    Vector integral_error_{Vector::Zero()};
    Vector previous_command_{Vector::Zero()};
};

extern template class PIDController<1>;
extern template class PIDController<2>;
extern template class PIDController<3>;

}  // namespace drone_node::algorithm
