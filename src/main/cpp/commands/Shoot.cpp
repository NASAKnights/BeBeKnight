// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Shooter *shooter, Indexer *indexer) : m_shooter{shooter},
                                                   m_indexer{indexer}
{
  AddRequirements(m_shooter);
  AddRequirements(m_indexer);
  finished = false;
}

// Called when the command is initially scheduled.
void Shoot::Initialize()
{
  m_shooter->SetSpeed(-0.8);
  timer.Start();
  timer.Reset();
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute()
{
  if (timer.HasElapsed(2_s) && m_indexer->hasBall())
  {
    m_indexer->moveIndexer();
    finished = true;
  }
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted)
{
  m_indexer->stopIndexer();
  m_shooter->Idle(0.0);
}

// Returns true when the command should end.
bool Shoot::IsFinished()
{
  return finished;
}
