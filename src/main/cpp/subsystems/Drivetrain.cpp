// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

// Drivetrain::Drivetrain() = default;

// This method will be called once per scheduler run
void Drivetrain::Periodic()
{

  m_poseEstimator.Update(m_gyro.GetRotation2d(),
                         units::meter_t{m_leftEncoder.GetDistance()},
                         units::meter_t{m_rightEncoder.GetDistance()});

  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
}

void Drivetrain::SimulationPeriodic()
{
  m_differentialsim.Update(20_ms);
  m_differentialsim.GetPose();

  m_differentialsim.SetInputs(
      m_leftLeader.Get() * units::volt_t(frc::RobotController::GetInputVoltage()),
      m_rightLeader.Get() * units::volt_t(frc::RobotController::GetInputVoltage()));

  m_leftEncoderSim.SetDistance(m_differentialsim.GetLeftPosition().value());
  // m_leftEncoderSim.SetRate(m_differentialsim.GetLeftVelocity().value());
  m_rightEncoderSim.SetDirection(m_differentialsim.GetRightPosition().value());
  // m_rightEncoderSim.SetRate(m_differentialsim.GetRightVelocity().value());
  m_gyroSim.SetAngle(double{-m_differentialsim.GetHeading().Degrees()});
  // m_differentialsim.getPose
}

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
  const auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  const auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  const double leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(), speeds.left.value());
  const double rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), speeds.right.value());
  if constexpr (frc::RobotBase::IsSimulation())
  {
    m_differentialsim.SetInputs(units::volt_t{leftOutput} + leftFeedforward,
                                units::volt_t{rightOutput} + rightFeedforward);
    return;
  }

  m_leftLeader.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightLeader.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(m_gyro.GetRotation2d(),
                    units::meter_t{m_leftEncoder.GetDistance()},
                    units::meter_t{m_rightEncoder.GetDistance()});
}
