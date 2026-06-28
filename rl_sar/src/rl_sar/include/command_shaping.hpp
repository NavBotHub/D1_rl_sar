/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMAND_SHAPING_HPP
#define COMMAND_SHAPING_HPP

#include <algorithm>
#include <cmath>
#include <vector>

namespace rl_command
{
constexpr float kBackwardCommandLimit = -0.7f;
constexpr float kForwardCommandLimit = 1.05f;
constexpr float kLateralCommandLimit = 0.6f;
constexpr float kYawCommandLimit = 0.6f;
constexpr float kYawStartCommand = 0.2f;
constexpr float kYawInputDeadband = 0.02f;
constexpr float kDiagonalXLimit = 0.5f;
constexpr float kDiagonalYLimit = 0.35f;
constexpr float kDiagonalActivationThreshold = 0.05f;
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

inline PlanarCommand ShapePlanarCommand(float raw_x, float raw_y)
{
    const float x_input = ClampFloat(raw_x, -1.0f, 1.0f);
    const float y_input = ClampFloat(raw_y, -1.0f, 1.0f);

    PlanarCommand cmd = {
        x_input >= 0.0f ? x_input * kForwardCommandLimit : x_input * -kBackwardCommandLimit,
        y_input * kLateralCommandLimit
    };

    if (std::abs(cmd.x) > kDiagonalActivationThreshold &&
        std::abs(cmd.y) > kDiagonalActivationThreshold)
    {
        const float ellipse =
            (cmd.x * cmd.x) / (kDiagonalXLimit * kDiagonalXLimit) +
            (cmd.y * cmd.y) / (kDiagonalYLimit * kDiagonalYLimit);
        if (ellipse > 1.0f)
        {
            const float scale = 1.0f / std::sqrt(ellipse);
            cmd.x *= scale;
            cmd.y *= scale;
        }
    }

    return cmd;
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

    return smoothed_commands;
}

} // namespace rl_command

#endif // COMMAND_SHAPING_HPP
