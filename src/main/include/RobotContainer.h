// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Drivetrain.h"
#include "commands/Shoot.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  void ConfigureBindings();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::Joystick m_driverController{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  Drivetrain m_driveTrain;
  Indexer m_indexer;
  Intake m_intake;

  Shooter m_shooter; // Add this line to declare the Shooter subsystem
};
