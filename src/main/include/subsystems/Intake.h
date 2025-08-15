// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

class Intake : public frc2::SubsystemBase
{
public:
  Intake();

  void intakeBall();
  void outakeBall();
  void stop();
  void startHopper();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::VictorSPX m_intakeMotor{1}; // can ID 1, change as needed
  ctre::phoenix::motorcontrol::can::VictorSPX m_hopperMotor{2}; // can ID 2, change as needed
};
