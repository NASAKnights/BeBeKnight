// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake()
{
    // set the motor to run at a slow speed (if needed)
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2); // 20% power
}

// This method will be called once per scheduler run
void Intake::Periodic()
{

    // ensure the motor continues running at the desired speed
    m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2); // 20% power
}
