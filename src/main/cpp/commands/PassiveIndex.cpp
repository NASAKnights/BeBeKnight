// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PassiveIndex.h"

PassiveIndex::PassiveIndex(Indexer *indexer) : m_indexer{indexer} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_indexer);

}

// Called when the command is initially scheduled.
void PassiveIndex::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PassiveIndex::Execute() {

  m_indexer->PassiveIndex();
  
}

// Called once the command ends or is interrupted.
void PassiveIndex::End(bool interrupted) {}

// Returns true when the command should end.
bool PassiveIndex::IsFinished() {
  return false;
}
