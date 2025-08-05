// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angle.h>
// #include <units/angular_velocity.h>
#include <units/length.h>
// #include <units/velocity.h>
#include <frc/AnalogGyro.h>
#include <numbers>
#include <units/voltage.h>
#include <units/math.h>
#include <Constants.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/RobotController.h>
#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/StateSpaceUtil.h"
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

class Drivetrain : public frc2::SubsystemBase
{
public:
  Drivetrain()
  {

    // We need to invert one side of the drive train so that the voltages result in both sides moving forward
    //(this is dependent on right needing to be flipped, but just switch if otherwise)
    m_rightLeader.SetInverted(true);

    frc::SmartDashboard::PutData("Field", &m_field);

    m_gyro.Reset();
    // Set the distance per pulse for the drive encoders. We can use the distance traveled for
    // one rotation of the wheel divided by the encoder resolution.
    m_leftEncoder.SetDistancePerPulse(2 * std::numbers::pi * DrivetrainConstants::WheelRadius.value() / DrivetrainConstants::EncoderResolution);
    m_rightEncoder.SetDistancePerPulse(2 * std::numbers::pi * DrivetrainConstants::WheelRadius.value() / DrivetrainConstants::EncoderResolution);

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
  }
  // static constexpr units::meters_per_second_t kMaxSpeed = DrivetrainConstants::MaxSpeed; // 1 meter per second
  // static constexpr units::radians_per_second_t kMaxAngularSpeed{
  //     std::numbers::pi}; // 1/2 rotation per second

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
  void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  void UpdateOdometry();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic();

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // frc::DCMotor driveMotor, double gearing, units::moment_of_inertia::kilogram_square_meter_t J,
  // units::mass::kilogram_t mass, units::length::meter_t wheelRadius,
  // units::length::meter_t trackWidth, const std::array<double, 7U> &measurementStdDevs = {}
  frc::DCMotor driveMotor = frc::DCMotor::CIM(1);

  frc::sim::DifferentialDrivetrainSim m_differentialsim{driveMotor, DrivetrainConstants::Gearing,
                                                        DrivetrainConstants::MOI, DrivetrainConstants::mass, DrivetrainConstants::WheelRadius, DrivetrainConstants::TrackWidth};

  frc::Field2d m_field;

  // frc::PWMSparkMax m_leftLeader{DrivetrainConstants::LeftLeaderSparkMaxId};
  // frc::PWMSparkMax m_rightLeader{DrivetrainConstants::RightLeaderSparkMaxId};

  ctre::phoenix::motorcontrol::can::VictorSPX m_leftLeader{4};  // CAN ID 4
  ctre::phoenix::motorcontrol::can::VictorSPX m_rightLeader{5}; // CAN ID 5

  frc::Encoder m_leftEncoder{DrivetrainConstants::LeftEncoder1, DrivetrainConstants::LeftEncoder2};
  frc::Encoder m_rightEncoder{DrivetrainConstants::RightEncoder1, DrivetrainConstants::RightEncoder2};

  frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};

  frc::PIDController m_leftPIDController{DrivetrainConstants::P, DrivetrainConstants::I, DrivetrainConstants::D};
  frc::PIDController m_rightPIDController{DrivetrainConstants::P, DrivetrainConstants::I, DrivetrainConstants::D};

  frc::AnalogGyro m_gyro{DrivetrainConstants::gyro};

  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};

  frc::DifferentialDriveKinematics m_kinematics{DrivetrainConstants::TrackWidth};
  frc::DifferentialDriveOdometry m_odometry{
      m_gyro.GetRotation2d(), units::meter_t{m_leftEncoder.GetDistance()},
      units::meter_t{m_rightEncoder.GetDistance()}};

  frc::SimpleMotorFeedforward<units::meters> m_feedforward{DrivetrainConstants::StaticFeedFoward, DrivetrainConstants::VelocityFeedFoward};

  frc::DifferentialDrivePoseEstimator m_poseEstimator{
      m_kinematics,
      m_gyro.GetRotation2d(),
      units::meter_t{m_leftEncoder.GetDistance()},
      units::meter_t{m_rightEncoder.GetDistance()},
      frc::Pose2d{},
      {0.01, 0.01, 0.01},
      {0.1, 0.1, 0.1}};
};
