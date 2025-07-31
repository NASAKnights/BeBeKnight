// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Indexer.h"
#include <frc/smartdashboard/SmartDashboard.h>

// This method will be called once per scheduler run

Indexer::Indexer() : limitSwitch(1) // Replace 1 with the actual digital input channel
{
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_BallLog = wpi::log::BooleanLogEntry(log, "/Indexer/Note");
}

void Indexer::Periodic()
{
    frc::SmartDashboard::PutBoolean("Has ball?", hasBall());
    m_BallLog.Append(hasBall());
}

bool Indexer::hasBall()
{
    return limitSwitch.Get();
}

void Indexer::moveIndexer()
{
    indexerMotor.Set(0.5); // Move the motor at 50% speed
}

void Indexer::stopIndexer()
{
    indexerMotor.Set(0.0); // Stop the motor
}