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
  m_intake.startHopper();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_driveTrain.SetDefaultCommand(frc2::RunCommand(
      [this]
      {
        auto leftAxis = -frc::ApplyDeadband(m_driverController.GetRawAxis(1), 0.1);
        auto rightAxis = -frc::ApplyDeadband(m_driverController.GetRawAxis(2), 0.1);
        m_driveTrain.Drive(leftAxis,
                           rightAxis);
      },
      {&m_driveTrain}));
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this]
  //               { return m_subsystem.ExampleCondition(); })
  //     .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  m_indexer.SetDefaultCommand(PassiveIndex(&m_indexer));

  

  frc2::JoystickButton(&m_driverController, 1)
      .WhileTrue(Shoot(&m_shooter, &m_indexer).ToPtr());

  frc2::JoystickButton(&m_driverController, 3)
      .WhileTrue(frc2::RunCommand([this]
                                  { m_intake.intakeBall(); },
                                  {&m_intake})
                     .ToPtr())
      .OnFalse(frc2::InstantCommand([this]
                                    { return m_intake.stop(); },
                                    {&m_intake})
                   .ToPtr());

  frc2::JoystickButton(&m_driverController, 4)
      .WhileTrue(frc2::RunCommand([this]
                                  { m_intake.outakeBall(); },
                                  {&m_intake})
                     .ToPtr())
      .OnFalse(frc2::InstantCommand([this]
                                    { return m_intake.stop(); },
                                    {&m_intake})
                   .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
