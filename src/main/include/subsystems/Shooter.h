// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>
#include "wpi/DataLog.h"
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <units/voltage.h>
#include <frc/Encoder.h>


namespace ShooterConstants
{
  enum ShooterState
  {
    Idle,
    SpinUp,
    Shoot
  };

  static constexpr int ShooterMotor = 1;
  static constexpr double MotorAtIdle = 0.0;
  static constexpr int ShooterEncoder = 1;

  const double kDriveP = 0.2;
  const double kDriveI = 0.0;
  const double kDriveD = 0.0;
  const double kDriveS = 0.02496863326; // Volts
  const double kDriveV = 0.1089791826;  // Volts / (rot / s)
  const double kDriveA = 0.0;           // Volts / (rot / s^2)

  // const double kDriveS = 0.05558; // Volts
  // const double kDriveV = 0.20333; // Volts / (rot / s)
  // const double kDriveA = 0.02250; // Volts / (rot / s^2)

  const double kSteerP = 35.0; // TODO: ensure this works with new inversion
  const double kSteerI = 0.0;
  const double kSteerD = 0.0;
}
class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Idle() {
    m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});
  }

private:

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_ShooterMotor{ShooterConstants::ShooterMotor};
  // frc::Encoder m_ShooterEncoder{ShooterConstants::ShooterEncoder};  
  

  

  ShooterConstants::ShooterState m_ShooterState;

  wpi::log::DoubleLogEntry m_VelocityLog;
  wpi::log::DoubleLogEntry m_VoltageLog;
  wpi::log::DoubleLogEntry m_CurrentLog;
};