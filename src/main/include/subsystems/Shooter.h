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
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/LinearSystem.h>

#include <frc/system/plant/DCMotor.h>

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

  const units::volt_t motorVoltage = units::volt_t{12.0};

}
class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Idle();       // Declaration only
  void Shoot();      // Declaration only
  void SpinUp();     // Declaration only
  double getSpeed(); // Declaration only

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_ShooterMotor{ShooterConstants::ShooterMotor};
  // frc::Encoder m_ShooterEncoder{ShooterConstants::ShooterEncoder};
  ctre::phoenix6::sim::TalonFXSimState *m_ShooterMotorSim = &m_ShooterMotor.GetSimState(); // simulate motor state
  static constexpr double kGearRatio = 10.0;                                               // TODO: ADD THIS AS A CONSTANT FOR SHOOTERS
  // frc::sim::DCMotorSim m_motorSimModel{m_ShooterMotor, kGearRatio, ShooterConstants::ShooterConstants::MOI,
  //                                      ShooterConstants::ShooterConstants::mass, ShooterConstants::ShooterConstants::WheelRadius,
  //                                      ShooterConstants::ShooterConstants::TrackWidth}; // simulate motor model

  frc::sim::DCMotorSim m_motorSimModel{frc::LinearSystemId::DCMotorSystem(frc::DCMotor::KrakenX60FOC(1), 0.001_kg_sq_m, kGearRatio), frc::DCMotor::KrakenX60FOC(1)};

  // ctre::m_motorSimModel

  frc::Encoder m_ShooterEncoder{ShooterConstants::ShooterEncoder, ShooterConstants::ShooterEncoder + 1}; // initialize encoder with two channels
  frc::sim::EncoderSim m_ShooterEncoderSim{m_ShooterEncoder};                                            // simulate encoder

  ShooterConstants::ShooterState m_ShooterState;

  wpi::log::DoubleLogEntry m_VelocityLog;
  wpi::log::DoubleLogEntry m_VoltageLog;
  wpi::log::DoubleLogEntry m_CurrentLog;
};