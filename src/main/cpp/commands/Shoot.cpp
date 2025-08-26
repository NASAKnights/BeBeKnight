// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Shooter *shooter, Indexer *indexer) : m_shooter{shooter},
                                                   m_indexer{indexer}
{
  AddRequirements(m_shooter);
  AddRequirements(m_indexer);
}

// Called when the command is initially scheduled.
void Shoot::Initialize()
{
  finished = false;
  m_shooter->SetSpeed(-0.9);
  timer1.Start();
  timer1.Reset();
  timer2.Start();
  timer2.Reset();
}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute()
{

  if (timer1.HasElapsed(2_s) && m_indexer->hasBall())
  {
    m_indexer->moveIndexer();
    // finished = true;
  } 
  else if(!m_indexer->hasBall())
  {
    timer1.Reset();
    m_indexer->moveIndexer();
  } 
  else if(m_indexer->hasBall() && !timer1.HasElapsed(2_s)){
    m_indexer->stopIndexer();
  } 
  else
  {
    m_indexer->moveIndexer();
  }
  // else {
  //   m_indexer->stopIndexer();
  // }
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
