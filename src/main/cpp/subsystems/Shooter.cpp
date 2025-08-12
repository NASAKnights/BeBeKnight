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
}

// This method will be called once per scheduler run
void Shooter::Periodic()
{
    switch (m_ShooterState)
    {
    case ShooterConstants::Idle:
        m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});
        // TODO: OUPUT THIS STATE TO SHUFFLEBOARD
        break;
    case ShooterConstants::SpinningUp:
        // Spin up motor to speed
        // SpinUp();

        break;
    case ShooterConstants::SpunUp:
        // Fire game object once up to speed and detected we have a game object in indexer

        break;
    }

    // Log real-world values
    // frc::SmartDashboard::PutNumber("Shooter Velocity", getSpeed());
}

void Shooter::Idle(double stopSpeed)
{
    // m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});

    m_leftShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, stopSpeed);
    m_rightShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, stopSpeed);
}

// void Shooter::SpinUp()
// {
//     m_ShooterMotor.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.8}); // Example: 80% power

//     // units::turns_per_second_t targetVelocity = units::turns_per_second_t{ShooterConstants::kShooterTargetVelocity};
//     // ctre::phoenix6::controls::VelocityDutyCycle velocityRequest{targetVelocity};
//     // m_ShooterMotor.SetControl(velocityRequest);
// }

void Shooter::SetSpeed(double speed)
{
    m_leftShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    m_rightShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

// double Shooter::getSpeed() {
//     return m_ShooterMotor.GetVelocity().GetValue();    //TODO most likely delete this function, as it uses the velocity,
// }                                                      //which we don't have access to due to the lack of encoders
