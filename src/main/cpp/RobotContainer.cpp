// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_driveTrain.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        auto leftAxis = -frc::ApplyDeadband(m_driverController.GetRawAxis(1), 0.1);
        auto rightAxis = -frc::ApplyDeadband(m_driverController.GetRawAxis(2), 0.1);
        m_driveTrain.Drive(units::meters_per_second_t{leftAxis} * 3,
                           units::radians_per_second_t{rightAxis});
      },
      {&m_driveTrain}));
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return m_subsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
