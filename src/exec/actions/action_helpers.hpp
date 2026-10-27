#pragma once

#include <chrono>
#include <cstdint>

#include "drone_node/msg/control_status.hpp"

namespace drone_node::exec
{

namespace
{

template<typename CacheT>
bool freshCache(
    const CacheT& cache,
    std::chrono::steady_clock::time_point now,
    double timeout)
{
    return cache.received &&
           cache.valid &&
           now >= cache.received_at &&
           std::chrono::duration<double>(
               now - cache.received_at).count() <= timeout;
}

inline double elapsedSince(
    std::chrono::steady_clock::time_point since)
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - since).count();
}

inline std::int64_t statusStamp(
    const drone_node::msg::ControlStatus& status)
{
    return static_cast<std::int64_t>(status.stamp.sec) *
               1000000000LL +
           static_cast<std::int64_t>(status.stamp.nanosec);
}

}  // namespace

}  // namespace drone_node::exec
