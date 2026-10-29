
#include "drone_node/ctrl/trajectory_manager.hpp"

#include <chrono>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace drone_node
{

TrajectoryManager::AcceptResult TrajectoryManager::accept(
    const ControlTrajectory& msg,
    SteadyTime now,
    std::string& reason)
{
    reason.clear();

    // Validate only the outer trajectory frame.
    // Keep the existing "map" frame convention.
    if (msg.frame_id != "map") {
        reason = "unsupported frame_id";
        return AcceptResult::REJECTED;
    }

    // Sampler replacement must not throw during move.
    // This guarantees that a rejected trajectory does not
    // destroy the currently active trajectory.
    static_assert(
        std::is_nothrow_move_constructible_v<Sampler>,
        "Sampler must be nothrow move constructible");

    try {
        // Construct and validate the new sampler locally.
        // The active sampler remains unchanged at this stage.
        Sampler candidate = [&]() -> Sampler {
            switch (msg.trajectory_type) {

            case ControlTrajectory::DISCRETE:
                return Sampler{
                    std::in_place_type<
                        trajectory_sampler::DiscreteSampler>,
                    msg.discrete_traj
                };

            case ControlTrajectory::PIECEWISE_POLYNOMIAL:
                return Sampler{
                    std::in_place_type<
                        trajectory_sampler::PolynomialSampler>,
                    msg.polynomial_traj
                };

            default:
                throw std::invalid_argument(
                    "unsupported trajectory_type");
            }
        }();

        // The candidate is now fully initialized.
        // Replace the previous trajectory.
        sampler_.emplace(std::move(candidate));

        // trajectory_id is metadata only.
        // No value restrictions or validity checks.
        trajectory_id_ = msg.trajectory_id;

        // New trajectory starts immediately.
        start_time_ = now;

        reason = "accepted";
        return AcceptResult::ACCEPTED;

    } catch (const std::exception& e) {
        // Validation or construction failed.
        // The previous trajectory remains active.
        reason = e.what();
        return AcceptResult::REJECTED;
    }
}

bool TrajectoryManager::has_trajectory() const noexcept
{
    return sampler_.has_value();
}

bool TrajectoryManager::is_started(
    SteadyTime now) const noexcept
{
    return sampler_.has_value() && now >= start_time_;
}

double TrajectoryManager::elapsed(
    SteadyTime now) const
{
    if (!sampler_) {
        throw std::logic_error(
            "No active trajectory");
    }

    if (now < start_time_) {
        throw std::invalid_argument(
            "Sample time precedes trajectory activation");
    }

    return std::chrono::duration<double>(
        now - start_time_).count();
}

ReferenceState TrajectoryManager::sample(
    SteadyTime now) const
{
    if (!sampler_) {
        throw std::logic_error(
            "No active sampler");
    }

    const double t = elapsed(now);

    return std::visit(
        [t](const auto& sampler) -> ReferenceState {
            return sampler.sample(t);
        },
        *sampler_);
}

TrajectoryManager::SampleResult
TrajectoryManager::sample_with_validity(
    SteadyTime now) const
{
    if (!sampler_) {
        throw std::logic_error(
            "No active sampler");
    }

    const double t = elapsed(now);

    return std::visit(
        [t](const auto& sampler) -> SampleResult {
            const auto sampled =
                sampler.sample_with_validity(t);

            SampleResult result;
            result.state = sampled.state;
            result.validity = sampled.validity;

            return result;
        },
        *sampler_);
}

double TrajectoryManager::duration() const
{
    if (!sampler_) {
        throw std::logic_error(
            "No active sampler");
    }

    return std::visit(
        [](const auto& sampler) -> double {
            return sampler.duration();
        },
        *sampler_);
}

uint64_t TrajectoryManager::trajectory_id() const noexcept
{
    return trajectory_id_;
}

void TrajectoryManager::clear() noexcept
{
    sampler_.reset();

    // Reset inactive metadata.
    // This does not imply that trajectory_id == 0
    // is invalid for an active trajectory.
    trajectory_id_ = 0;
    start_time_ = SteadyTime{};
}

void TrajectoryManager::activate(
    SteadyTime now)
{
    if (!sampler_) {
        throw std::logic_error(
            "Cannot activate an empty trajectory");
    }

    // Reset the time origin without rebuilding the sampler.
    start_time_ = now;
}

}  // namespace drone_node
