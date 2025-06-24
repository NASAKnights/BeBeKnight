// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include "wpi/DataLog.h"
namespace ShooterConstants
{
  enum ShooterState
  {
    Idle,
    SpinUp,
    Shoot
  };
};
class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Idle();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_ShooterMoter;

  ShooterConstants::ShooterState m_ShooterState;

  wpi::log::DoubleLogEntry m_VelocityLog;
  wpi::log::DoubleLogEntry m_VoltageLog;
  wpi::log::DoubleLogEntry m_CurrentLog;
};
