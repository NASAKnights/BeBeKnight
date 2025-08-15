// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
// : m_ShooterMotor(ShooterConstants::ShooterMotor),  // Initialize TalonFX motor with CAN ID
//   m_ShooterMotorSim(&m_ShooterMotor.GetSimState()) // Initialize simulation state
{

    // Additional initialization logic if needed

    // auto &talonFXSim = m_talonFX.GetSimState();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.SetInputVoltage(ShooterConstants::motorVoltage);
    m_motorSimModel.Update(20_ms); // assume 20 ms loop time

    m_leftShooterMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);
    m_rightShooterMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Coast);

    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration shooterMotorConfig;
    shooterMotorConfig.currentLimit = 35;
    shooterMotorConfig.triggerThresholdCurrent = 40;
    shooterMotorConfig.triggerThresholdTime = 0.1;

    m_leftShooterMotor.ConfigSupplyCurrentLimit(shooterMotorConfig);
    m_rightShooterMotor.ConfigSupplyCurrentLimit(shooterMotorConfig);
    m_leftShooterMotor.EnableCurrentLimit(true);
    m_rightShooterMotor.EnableCurrentLimit(true);

    m_rightShooterMotor.Follow(m_leftShooterMotor);
}

// This method will be called once per scheduler run
void Shooter::Periodic()
{

    // Log real-world values
    // frc::SmartDashboard::PutNumber("Shooter Velocity", getSpeed());
}

void Shooter::Idle(double shootSpeed)
{
    // m_ShooterMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{ShooterConstants::MotorAtIdle}});

    m_leftShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, shootSpeed);
}

void Shooter::SpinUp()
{
    m_leftShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.8); // Example: 80% power
}

void Shooter::SetSpeed(double speed)
{
    m_leftShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    // m_rightShooterMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}
