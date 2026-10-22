#pragma once

#include <cmath>
#include <stdexcept>

#include "drone_node/ctrl/state_cache.hpp"

namespace drone_node
{

class HoverReference
{
public:
    struct Config
    {
        double speed_threshold{0.1};      // m/s
        double yaw_rate_threshold{0.1};   // rad/s
        double settle_duration{0.3};      // s
    };

    enum class Phase
    {
        BRAKING,
        LOW_SPEED,
        HOLD
    };

    explicit HoverReference(const Config& config)
        : config_(config)
    {
        if (!positive(config_.speed_threshold) ||
            !positive(config_.yaw_rate_threshold) ||
            !positive(config_.settle_duration)) {
            throw std::invalid_argument(
                "Invalid hover reference configuration");
        }
    }

    void start(const StateSnapshot& state)
    {
        validate(state);

        reference_ = current_reference(state);
        phase_ = Phase::BRAKING;
        last_time_ = state.captured_at;
        low_speed_since_ = SteadyTime{};
        active_ = true;
    }

    ReferenceState sample(const StateSnapshot& state)
    {
        if (!active_) {
            throw std::logic_error("Hover reference not started");
        }

        validate(state);

        if (state.captured_at < last_time_) {
            throw std::invalid_argument(
                "Hover reference time moved backwards");
        }

        last_time_ = state.captured_at;

        if (phase_ == Phase::HOLD) {
            return reference_;
        }

        // During braking, do not pull back toward the CANCEL position.
        // The physical controller receives zero position/yaw error
        // and zero desired linear/angular velocity.
        reference_ = current_reference(state);

        const bool low_speed =
            state.velocity.norm() <= config_.speed_threshold &&
            std::abs(state.measured_yaw_rate) <=
                config_.yaw_rate_threshold;

        if (!low_speed) {
            phase_ = Phase::BRAKING;
            return reference_;
        }

        if (phase_ == Phase::BRAKING) {
            phase_ = Phase::LOW_SPEED;
            low_speed_since_ = state.captured_at;
        }

        const double settled = std::chrono::duration<double>(
            state.captured_at - low_speed_since_).count();

        if (settled >= config_.settle_duration) {
            // Freeze the actual pose at the end of deceleration.
            phase_ = Phase::HOLD;
        }

        return reference_;
    }

    Phase phase() const noexcept
    {
        return phase_;
    }

private:
    static bool positive(double value)
    {
        return std::isfinite(value) && value > 0.0;
    }

    static void validate(const StateSnapshot& state)
    {
        if (!StateCache::usable(state)) {
            throw std::invalid_argument(
                "Hover reference requires valid feedback");
        }
    }

    static ReferenceState current_reference(
        const StateSnapshot& state)
    {
        ReferenceState reference;
        reference.position = state.position;
        reference.yaw = state.unwrapped_yaw;
        return reference;
    }

    Config config_;
    bool active_{false};
    Phase phase_{Phase::BRAKING};

    SteadyTime last_time_{};
    SteadyTime low_speed_since_{};
    ReferenceState reference_{};
};

}  // namespace drone_node