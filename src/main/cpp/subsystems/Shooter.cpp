// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
    : m_ShooterMotor(ShooterConstants::ShooterMotor),  // Initialize TalonFX motor with CAN ID
      m_ShooterMotorSim(&m_ShooterMotor.GetSimState()) // Initialize simulation state
{

    // Additional initialization logic if needed

    // auto &talonFXSim = m_talonFX.GetSimState();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.SetInputVoltage(ShooterConstants::motorVoltage);
    m_motorSimModel.Update(20_ms); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    m_ShooterMotorSim->SetRawRotorPosition(kGearRatio * m_motorSimModel.GetAngularPosition());
    m_ShooterMotorSim->SetRotorVelocity(kGearRatio * m_motorSimModel.GetAngularVelocity());
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
    frc::SmartDashboard::PutNumber("Simulated Shooter Velocity", (kGearRatio * m_motorSimModel.GetAngularVelocity()).value());
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
    // frc::SmartDashboard::PutNumber("Shooter Velocity", getSpeed());
}

void Shooter::Idle()
{
    m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});
}

void Shooter::Shoot()
{
    // Implementation for Shoot (currently empty)
}

void Shooter::SpinUp()
{
    m_ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.8}); // Example: 80% power

    // units::turns_per_second_t targetVelocity = units::turns_per_second_t{ShooterConstants::kShooterTargetVelocity};
    // ctre::phoenix6::controls::VelocityDutyCycle velocityRequest{targetVelocity};
    // m_ShooterMotor.SetControl(velocityRequest);
}

// double Shooter::getSpeed() {
//     return m_ShooterMotor.GetVelocity().GetValue();    //TODO most likely delete this function, as it uses the velocity,
// }                                                      //which we don't have access to due to the lack of encoders
