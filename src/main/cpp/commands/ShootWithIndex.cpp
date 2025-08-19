// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootWithIndex.h"

ShootWithIndex::ShootWithIndex(Shooter *shooter, Indexer *indexer) : m_shooter{shooter},
                                                   m_indexer{indexer}
{
  AddRequirements(m_shooter);
  AddRequirements(m_indexer);
}

// Called when the command is initially scheduled.
void ShootWithIndex::Initialize() {



}

// Called repeatedly when this Command is scheduled to run
void ShootWithIndex::Execute() {



}

// Called once the command ends or is interrupted.
void ShootWithIndex::End(bool interrupted) {



}

// Returns true when the command should end.
bool ShootWithIndex::IsFinished() {
  return false;
}
