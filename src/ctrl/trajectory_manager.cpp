
#include "drone_node/ctrl/trajectory_manager.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "drone_node/ctrl/trajectory_sampler/discrete_sampler.hpp"
#include "drone_node/ctrl/trajectory_sampler/polynomial_sampler.hpp"

namespace drone_node
{

TrajectoryManager::AcceptResult TrajectoryManager::prepare(
    const DiscreteTraj& msg,
    std::string& reason)
{
    reason.clear();

    if (msg.frame_id != "map") {
        reason = "unsupported frame_id";
        return AcceptResult::REJECTED;
    }

    try {
        // Construction performs validation and preprocessing.
        auto candidate =
            std::make_unique<trajectory_sampler::DiscreteSampler>(
                msg);

        // Update the prepared slot only after successful
        // construction. The active trajectory is unchanged.
        prepared_sampler_ = std::move(candidate);
        prepared_kind_ = SamplerKind::DISCRETE;
        prepared_trajectory_id_ = msg.trajectory_id;

        reason = "accepted";
        return AcceptResult::ACCEPTED;
    } catch (const std::exception& e) {
        reason = e.what();
        return AcceptResult::REJECTED;
    }
}

TrajectoryManager::AcceptResult TrajectoryManager::prepare(
    const PolynomialTraj& msg,
    std::string& reason)
{
    reason.clear();

    if (msg.frame_id != "map") {
        reason = "unsupported frame_id";
        return AcceptResult::REJECTED;
    }

    try {
        // Construction validates coefficients, durations
        // and inter-segment continuity.
        auto candidate =
            std::make_unique<trajectory_sampler::PolynomialSampler>(
                msg);

        // Do not replace the active sampler yet.
        prepared_sampler_ = std::move(candidate);
        prepared_kind_ = SamplerKind::POLYNOMIAL;
        prepared_trajectory_id_ = msg.trajectory_id;

        reason = "accepted";
        return AcceptResult::ACCEPTED;
    } catch (const std::exception& e) {
        reason = e.what();
        return AcceptResult::REJECTED;
    }
}

bool TrajectoryManager::commit(SteadyTime now) noexcept
{
    if (!prepared_sampler_) {
        return false;
    }

    // No validation or sampler construction occurs here.
    // Transfer ownership of the prepared sampler.
    sampler_ = std::move(prepared_sampler_);

    sampler_kind_ = prepared_kind_;
    trajectory_id_ = prepared_trajectory_id_;

    start_time_ = now;
    has_trajectory_ = true;

    // Reset prepared metadata after the transfer.
    prepared_kind_ = SamplerKind::NONE;
    prepared_trajectory_id_ = 0;

    return true;
}

void TrajectoryManager::discard_prepared() noexcept
{
    prepared_sampler_.reset();

    prepared_kind_ = SamplerKind::NONE;
    prepared_trajectory_id_ = 0;
}

bool TrajectoryManager::has_prepared() const noexcept
{
    return prepared_sampler_ != nullptr;
}

bool TrajectoryManager::has_trajectory() const noexcept
{
    return has_trajectory_;
}

bool TrajectoryManager::is_started(SteadyTime now) const noexcept
{
    return has_trajectory_ && now >= start_time_;
}

double TrajectoryManager::elapsed(SteadyTime now) const
{
    if (!has_trajectory_) {
        throw std::logic_error("No active trajectory");
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
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    return sampler_->sample(elapsed(now));
}

TrajectoryManager::SampleResult
TrajectoryManager::sample_with_validity(
    SteadyTime now) const
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    const double t = elapsed(now);

    SampleResult result;

    switch (sampler_kind_) {
    case SamplerKind::DISCRETE: {
        const auto* discrete =
            static_cast<
                const trajectory_sampler::DiscreteSampler*>(
                    sampler_.get());

        const auto sampled =
            discrete->sample_with_validity(t);

        result.state = sampled.state;
        result.validity = sampled.validity;

        return result;
    }

    case SamplerKind::POLYNOMIAL: {
        const auto* polynomial =
            static_cast<
                const trajectory_sampler::PolynomialSampler*>(
                    sampler_.get());

        const auto sampled =
            polynomial->sample_with_validity(t);

        result.state = sampled.state;
        result.validity = sampled.validity;

        return result;
    }

    case SamplerKind::NONE:
    default:
        throw std::logic_error(
            "Unknown active sampler type");
    }
}

double TrajectoryManager::duration() const
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error("No active sampler");
    }

    return sampler_->duration();
}

uint64_t TrajectoryManager::trajectory_id() const noexcept
{
    return trajectory_id_;
}

void TrajectoryManager::clear() noexcept
{
    // Clear the active trajectory.
    sampler_.reset();
    sampler_kind_ = SamplerKind::NONE;

    trajectory_id_ = 0;
    start_time_ = SteadyTime{};
    has_trajectory_ = false;

    // Clear any pending candidate.
    discard_prepared();
}

void TrajectoryManager::activate(SteadyTime now)
{
    if (!has_trajectory_ || !sampler_) {
        throw std::logic_error(
            "Cannot activate an empty trajectory");
    }

    start_time_ = now;
}

}  // namespace drone_node
