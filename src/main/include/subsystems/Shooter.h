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
#include <units/velocity.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/LinearSystemSim.h>

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

  const double kShooterTargetVelocity = 50;
}
class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic();

  void Idle()
  {
    m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});
  }
  // This sets the voltage to zero and thus idles the motor

  void Shoot()
  {
  }

  void SpinUp()
  {
    units::turns_per_second_t targetVelocity = units::turns_per_second_t{ShooterConstants::kShooterTargetVelocity};
    ctre::phoenix6::controls::VelocityDutyCycle velocityRequest{targetVelocity};
    m_ShooterMotor.SetControl(velocityRequest);
  }
  // this

  double getSpeed()
  {
    m_ShooterMotor.GetVelocity().GetValue();
  }
  // this gets the speed

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_ShooterMotor{ShooterConstants::ShooterMotor};
  // frc::Encoder m_ShooterEncoder{ShooterConstants::ShooterEncoder};
  ctre::phoenix6::sim::TalonFXSimState *m_ShooterMotorSim = &m_ShooterMotor.GetSimState(); // simulate motor state
  static constexpr double kGearRatio = 10.0;                                               // TODO: ADD THIS AS A CONSTANT FOR SHOOTERS
  frc::sim::DCMotorSim m_motorSimModel{m_ShooterMotor, kGearRatio, ShooterConstants::ShooterConstants::MOI,
                                       ShooterConstants::ShooterConstants::mass, ShooterConstants::ShooterConstants::WheelRadius,
                                       ShooterConstants::ShooterConstants::TrackWidth}; // simulate motor model

  // ctre::m_motorSimModel

  frc::Encoder m_ShooterEncoder{ShooterConstants::ShooterEncoder, ShooterConstants::ShooterEncoder + 1}; // initialize encoder with two channels
  frc::sim::EncoderSim m_ShooterEncoderSim{m_ShooterEncoder};                                            // simulate encoder

  ShooterConstants::ShooterState m_ShooterState;

  wpi::log::DoubleLogEntry m_VelocityLog;
  wpi::log::DoubleLogEntry m_VoltageLog;
  wpi::log::DoubleLogEntry m_CurrentLog;
};