// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
#include <units/velocity.h>
#include <units/voltage.h>
#include <numbers>
#include <units/moment_of_inertia.h>

namespace OperatorConstants
{

    inline constexpr int kDriverControllerPort = 0;

} // namespace OperatorConstants

namespace DrivetrainConstants
{

    static constexpr auto MaxSpeed = 1.0_mps;
    static constexpr auto TrackWidth = 0.381_m * 2;
    static constexpr auto WheelRadius = 0.0508_m; // meters
    static constexpr int EncoderResolution = 4096;
    static constexpr int LeftLeaderSparkMaxId = 1;
    static constexpr int RightLeaderSparkMaxId = 2;
    static constexpr int LeftEncoder1 = 0;
    static constexpr int LeftEncoder2 = 1;
    static constexpr int RightEncoder1 = 2;
    static constexpr int RightEncoder2 = 3;
    static constexpr double P = 1.0;
    static constexpr double I = 0.0;
    static constexpr double D = 0.0;
    static constexpr int gyro = 0;
    static constexpr auto StaticFeedFoward = 1_V;
    static constexpr auto VelocityFeedFoward = 3_V / 1_mps;
    static constexpr double Gearing = 12.0;
    static constexpr units::moment_of_inertia::kilogram_square_meter_t MOI = units::moment_of_inertia::kilogram_square_meter_t(0.1);
    static constexpr units::mass::kilogram_t mass{60};

}
