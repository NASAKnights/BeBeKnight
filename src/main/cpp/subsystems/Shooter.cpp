// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter()
{
}

// This method will be called once per scheduler run
void Shooter::Periodic()
{
    switch (m_ShooterState)
    {
    case ShooterConstants::Idle:
        Idle();
        break;
    case ShooterConstants::SpinUp:
        // Spin up motor to speed
        break;
    case ShooterConstants::Shoot:
        // Fire game object once up to speed and detected we have a game object in indexer
        break;
    }
}

void Shooter::Idle()
{
}