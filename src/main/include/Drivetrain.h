// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <wpi/sendable/SendableRegistry.h>


/**
 * Represents a mecanum drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    m_gyro.Reset();
   
     //void 	EnsureInitialized ();
     //void   SetLiveWindowBuilderFactory (std::function< std::unique_ptr< SendableBuilder >()> factory);
     //void   Add (Sendable *sendable, std::string_view name);

     //wpi::SendableRegistry::AddChild(&m_kinematics, &m_frontLeftMotor);
     //wpi::SendableRegistry::AddChild(&m_kinematics, &m_rearLeftMotor);
     //wpi::SendableRegistry::AddChild(&m_kinematics, &m_frontRightMotor);
     //wpi::SendableRegistry::AddChild(&m_kinematics, &m_rearRightMotor);

     // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_frontRightMotor.SetInverted(true);
    m_rearRightMotor.SetInverted(true);
  }

  frc::MecanumDriveWheelSpeeds GetCurrentState() const;
  frc::MecanumDriveWheelPositions GetCurrentDistances() const;
  void SetSpeeds(const frc::MecanumDriveWheelSpeeds& wheelSpeeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void UpdateOdometry();

  static constexpr auto kMaxSpeed = 3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      std::numbers::pi};  // 1/2 rotation per second
      


 //static WheelSpeeds  DriveCartesianIK (double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle = 0_rad);

 private:
  frc::PWMSparkMax m_frontLeftMotor{6};
  frc::PWMSparkMax m_frontRightMotor{1};
  frc::PWMSparkMax m_rearLeftMotor{2};
  frc::PWMSparkMax m_rearRightMotor{3};

  //frc::RevSparkMax m_frontLeftMotor{};


  frc::Encoder m_frontLeftEncoder{0, 1};
  frc::Encoder m_frontRightEncoder{2, 3};
  frc::Encoder m_backLeftEncoder{4, 5};
  frc::Encoder m_backRightEncoder{6, 7};

  frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
  frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
  frc::Translation2d m_rearLeftLocation{-0.381_m, 0.381_m};
  frc::Translation2d m_rearRightLocation{-0.381_m, -0.381_m};

  frc::PIDController m_frontLeftPIDController{1.0, 0.0, 0.0};
  frc::PIDController m_frontRightPIDController{1.0, 0.0, 0.0};
  frc::PIDController m_rearLeftPIDController{1.0, 0.0, 0.0};
  frc::PIDController m_rearRightPIDController{1.0, 0.0, 0.0};

  frc::AnalogGyro m_gyro{0};
 

  frc::MecanumDriveKinematics m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation,
      m_rearRightLocation};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::MecanumDrivePoseEstimator m_poseEstimator{
      m_kinematics,  m_gyro.GetRotation2d(), GetCurrentDistances(),
      frc::Pose2d{}, {0.1, 0.1, 0.1},        {0.1, 0.1, 0.1}};
};
