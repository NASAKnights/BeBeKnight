// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter()
    : m_ShooterMotor(ShooterConstants::ShooterMotor),  // Initialize TalonFX motor with CAN ID
      m_ShooterMotorSim(&m_ShooterMotor.GetSimState()) // Initialize simulation state
{
    // Additional initialization logic if needed
}

// This method will be called once per scheduler run
void Shooter::Periodic()
{
    // Update the simulated motor state
    m_ShooterMotorSim->SetSupplyVoltage(m_ShooterMotor.GetSupplyVoltage().GetValue());

    // Update the encoder simulation based on the motor velocity

    m_ShooterEncoderSim.SetRate(m_ShooterMotor.GetVelocity().GetValueAsDouble());
    m_motorSimModel.SetInputVoltage(m_ShooterMotor.GetSupplyVoltage().GetValue());
    m_motorSimModel.Update(20_ms); // assume 20 ms loop time

    // Log simulated values to SmartDashboard
    frc::SmartDashboard::PutNumber("Simulated Shooter Velocity", m_ShooterMotorSim..value());
    frc::SmartDashboard::PutNumber("Simulated Shooter Voltage", m_ShooterMotorSim->GetMotorVoltage().value());

    switch (m_ShooterState)
    {
    case ShooterConstants::Idle:
        Idle();
        break;
    case ShooterConstants::SpinUp:
        // Spin up motor to speed
        SpinUp();
        break;
    case ShooterConstants::Shoot:
        // Fire game object once up to speed and detected we have a game object in indexer
        Shoot();
        break;
    }

    // Log real-world values
    frc::SmartDashboard::PutNumber("Shooter Velocity", getSpeed());
}
