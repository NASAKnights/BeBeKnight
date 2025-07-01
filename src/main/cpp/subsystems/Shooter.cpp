// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(){
    configs::Slot0Configs driveSlot0Configs{};
    configs::Slot0Configs steerSlot0Configs{};
    driveSlot0Configs.kP = kDriveP;
    driveSlot0Configs.kI = kDriveI;
    driveSlot0Configs.kD = kDriveD;
    driveSlot0Configs.kS = kDriveS;
    driveSlot0Configs.kV = kDriveV;
    driveSlot0Configs.kA = kDriveA;
    steerSlot0Configs.kP = kSteerP;
    steerSlot0Configs.kI = kSteerI;
    steerSlot0Configs.kD = kSteerD;
    driveConfig.Slot0 = driveSlot0Configs;
    steerConfig.Slot0 = steerSlot0Configs;

    configs::CurrentLimitsConfigs driveCurrentLimitConfig{};
    configs::CurrentLimitsConfigs steerCurrentLimitConfig{};
    driveCurrentLimitConfig.SupplyCurrentLimitEnable = kDriveEnableCurrentLimit;
    driveCurrentLimitConfig.SupplyCurrentLimit = kDriveContinuousCurrentLimit;
    driveCurrentLimitConfig.SupplyCurrentThreshold = kDrivePeakCurrentLimit;
    driveCurrentLimitConfig.SupplyTimeThreshold = kDrivePeakCurrentDuration;
    steerCurrentLimitConfig.SupplyCurrentLimitEnable = kSteerEnableCurrentLimit;
    steerCurrentLimitConfig.SupplyCurrentLimit = kSteerContinuousCurrentLimit;
    steerCurrentLimitConfig.SupplyCurrentThreshold = kSteerPeakCurrentLimit;
    steerCurrentLimitConfig.SupplyTimeThreshold = kSteerPeakCurrentDuration;
    driveConfig.CurrentLimits = driveCurrentLimitConfig;
    steerConfig.CurrentLimits = steerCurrentLimitConfig;
};


// This method will be called once per scheduler run
void Shooter::Periodic()
{
    switch (m_ShooterState)
    {
    case ShooterConstants::Idle:
        Idle();
        break;
    case ShooterConstants::SpinUp:
        // Spin up motor to speed
        break;
    case ShooterConstants::Shoot:
        // Fire game object once up to speed and detected we have a game object in indexer
        break;
    }
    m_ShooterMotor.GetVelocity();
}

double Shooter::getSpeed()
{
    
    return m_ShooterEncoder.GetVelocity();
}


// void Shooter::Idle()
// {
// }