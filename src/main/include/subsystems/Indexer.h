// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/DataLogManager.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <wpi/DataLog.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

class Indexer : public frc2::SubsystemBase
{
public:
  Indexer();

  /**
   * Called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Moves the indexer mechanism.
   */
  void moveIndexer();

  /**
   * Stops the indexer mechanism.
   */
  void stopIndexer();

  /**
   * Checks if a ball is detected by the limit switch.
   * @return True if a ball is detected, false otherwise.
   */
  bool hasBall();

private:
  frc::DigitalInput limitSwitch;
  ctre::phoenix::motorcontrol::can::VictorSPX m_indexerMotor{3}; // can ID 3, change as needed
  wpi::log::BooleanLogEntry m_BallLog;
};
