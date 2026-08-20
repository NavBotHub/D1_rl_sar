/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMAND_SHAPING_HPP
#define COMMAND_SHAPING_HPP

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

namespace rl_command
{
constexpr float kBackwardCommandLimit = -0.4f;
constexpr float kForwardCommandLimit = 0.5f;
constexpr float kLateralCommandLimit = 0.4f;
constexpr float kYawCommandLimit = 0.5f;
constexpr float kYawStartCommand = 0.2f;
constexpr float kYawInputDeadband = 0.02f;
constexpr float kForwardDiagonalXLimit = 0.5f;
constexpr float kForwardDiagonalYLimit = 0.35f;
constexpr float kBackwardDiagonalXLimit = 0.4f;
constexpr float kBackwardDiagonalYLimit = 0.25f;
constexpr float kDiagonalActivationThreshold = 0.05f;
constexpr float kMixedYawForwardXLimit = 0.5f;
constexpr float kMixedYawForwardYawLimit = 0.5f;
// Keep backward mixed-yaw disabled under the narrowed -0.4 m/s envelope until
// an accepted training anchor inside the new range is audited.
constexpr float kMixedYawBackwardLowerXLimit = -0.6f;
constexpr float kMixedYawBackwardUpperXLimit = -0.5f;
constexpr float kMixedYawBackwardYawLimit = 0.4f;
constexpr float kCommandAccelX = 1.6f;
constexpr float kCommandDecelX = 2.4f;
constexpr float kCommandAccelY = 1.2f;
constexpr float kCommandDecelY = 1.8f;
constexpr float kCommandAccelYaw = 2.0f;
constexpr float kCommandDecelYaw = 3.0f;

struct PlanarCommand
{
    float x;
    float y;
};

inline float ClampFloat(float value, float low, float high)
{
    return std::max(low, std::min(value, high));
}

inline PlanarCommand LimitPlanarDiagonal(PlanarCommand cmd)
{
    if (std::abs(cmd.x) > kDiagonalActivationThreshold &&
        std::abs(cmd.y) > kDiagonalActivationThreshold)
    {
        if (cmd.x >= 0.0f)
        {
            cmd.x = ClampFloat(cmd.x, 0.0f, kForwardDiagonalXLimit);
            cmd.y = ClampFloat(cmd.y, -kForwardDiagonalYLimit, kForwardDiagonalYLimit);
        }
        else
        {
            cmd.x = ClampFloat(cmd.x, -kBackwardDiagonalXLimit, 0.0f);
            cmd.y = ClampFloat(cmd.y, -kBackwardDiagonalYLimit, kBackwardDiagonalYLimit);
        }
    }

    return cmd;
}

inline float MixedYawLimitForTranslation(float x)
{
    if (x > kDiagonalActivationThreshold && x <= kMixedYawForwardXLimit)
    {
        return kMixedYawForwardYawLimit;
    }
    if (x >= kMixedYawBackwardLowerXLimit && x <= kMixedYawBackwardUpperXLimit)
    {
        return kMixedYawBackwardYawLimit;
    }
    return 0.0f;
}

inline void LimitUnsupportedCommandMix(std::vector<float>& commands)
{
    const bool has_x = std::abs(commands[0]) > kDiagonalActivationThreshold;
    const bool has_y = std::abs(commands[1]) > kDiagonalActivationThreshold;
    const bool has_yaw = std::abs(commands[2]) > kYawInputDeadband;

    if (!has_yaw)
    {
        return;
    }

    if (!has_x && !has_y)
    {
        return;
    }

    if (has_y)
    {
        commands[2] = 0.0f;
        return;
    }

    const float yaw_limit = MixedYawLimitForTranslation(commands[0]);
    commands[2] = ClampFloat(commands[2], -yaw_limit, yaw_limit);
}

inline PlanarCommand ShapePlanarCommand(float raw_x, float raw_y)
{
    const float x_input = ClampFloat(raw_x, -1.0f, 1.0f);
    const float y_input = ClampFloat(raw_y, -1.0f, 1.0f);

    PlanarCommand cmd = {
        x_input >= 0.0f ? x_input * kForwardCommandLimit : x_input * -kBackwardCommandLimit,
        y_input * kLateralCommandLimit
    };

    return LimitPlanarDiagonal(cmd);
}

inline float ShapeYawCommand(float raw_yaw)
{
    const float yaw_input = ClampFloat(raw_yaw, -1.0f, 1.0f);
    const float mag = std::abs(yaw_input);
    if (mag <= kYawInputDeadband)
    {
        return 0.0f;
    }

    const float normalized = (mag - kYawInputDeadband) / (1.0f - kYawInputDeadband);
    const float shaped = kYawStartCommand + normalized * (kYawCommandLimit - kYawStartCommand);
    return yaw_input < 0.0f ? -shaped : shaped;
}

inline std::vector<float> ClampCommands(const std::vector<float>& target_commands)
{
    if (target_commands.size() < 3)
    {
        return target_commands;
    }

    std::vector<float> commands = target_commands;
    commands[0] = ClampFloat(commands[0], kBackwardCommandLimit, kForwardCommandLimit);
    commands[1] = ClampFloat(commands[1], -kLateralCommandLimit, kLateralCommandLimit);
    commands[2] = ClampFloat(commands[2], -kYawCommandLimit, kYawCommandLimit);

    PlanarCommand planar = {commands[0], commands[1]};
    planar = LimitPlanarDiagonal(planar);
    commands[0] = planar.x;
    commands[1] = planar.y;
    LimitUnsupportedCommandMix(commands);
    return commands;
}

inline float SlewToward(float current, float target, float accel_rate, float decel_rate, float dt)
{
    const float delta = target - current;
    if (std::abs(delta) <= 1.0e-6f || dt <= 0.0f)
    {
        return target;
    }

    const bool same_sign = current * target > 0.0f;
    const bool crossing_zero = current * target < 0.0f;
    const bool reducing_magnitude = same_sign && std::abs(target) < std::abs(current);
    const bool moving_to_zero = std::abs(target) <= 1.0e-6f;
    const float rate = (crossing_zero || reducing_magnitude || moving_to_zero) ? decel_rate : accel_rate;
    const float max_delta = std::max(rate * dt, 0.0f);

    return current + ClampFloat(delta, -max_delta, max_delta);
}

inline std::vector<float> SmoothCommands(const std::vector<float>& target_commands,
                                         float dt,
                                         std::vector<float>& smoothed_commands,
                                         bool& smoothing_initialized,
                                         bool& smoothing_reset_requested)
{
    if (target_commands.size() < 3)
    {
        return target_commands;
    }

    if (smoothed_commands.size() != 3)
    {
        smoothed_commands.assign(3, 0.0f);
    }

    if (!smoothing_initialized)
    {
        smoothed_commands.assign(3, 0.0f);
        smoothing_initialized = true;
    }

    if (smoothing_reset_requested)
    {
        smoothed_commands.assign(3, 0.0f);
        smoothing_reset_requested = false;
    }

    smoothed_commands[0] = SlewToward(
        smoothed_commands[0], target_commands[0], kCommandAccelX, kCommandDecelX, dt);
    smoothed_commands[1] = SlewToward(
        smoothed_commands[1], target_commands[1], kCommandAccelY, kCommandDecelY, dt);
    smoothed_commands[2] = SlewToward(
        smoothed_commands[2], target_commands[2], kCommandAccelYaw, kCommandDecelYaw, dt);

    // A component-wise slew can temporarily enter an unsupported mixed region
    // even when both endpoints are valid (for example diagonal -> pure forward,
    // or forward+yaw -> higher forward). Project the final command back through
    // the same deployment envelope and synchronize the smoother state so the
    // next step cannot integrate from an unsafe intermediate value.
    const std::vector<float> projected_commands = ClampCommands(smoothed_commands);
    smoothed_commands.assign(projected_commands.begin(), projected_commands.begin() + 3);
    return projected_commands;
}

inline void LogCommandTrace(const std::string& source,
                            const std::string& raw_units,
                            const std::vector<float>& raw,
                            const std::vector<float>& shaped,
                            const std::vector<float>& smoothed)
{
    if (raw.size() < 3 || shaped.size() < 3 || smoothed.size() < 3)
    {
        return;
    }
    static auto last_log = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (last_log.time_since_epoch().count() != 0 && now - last_log < std::chrono::seconds(1))
    {
        return;
    }
    last_log = now;
    std::cout << "[command_trace] source=" << source << " raw_units=" << raw_units
              << " raw=[" << raw[0] << "," << raw[1] << "," << raw[2] << "]"
              << " shaped_mps=[" << shaped[0] << "," << shaped[1] << "," << shaped[2] << "]"
              << " smoothed_mps=[" << smoothed[0] << "," << smoothed[1] << "," << smoothed[2] << "]"
              << std::endl;
}

} // namespace rl_command

#endif // COMMAND_SHAPING_HPP
