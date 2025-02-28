// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <numbers>
//#include "AHRS.h"
#include <frc/TimedRobot.h>
//#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/GenericHID.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>


#include <frc/motorcontrol/Spark.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMMotorController.h>

#include <cstdio>
#include <cameraserver/CameraServer.h>
//#include <frc/apriltag/AprilTagPoseEstimate.h>

#include <frc/RobotController.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <rev/SparkMax.h>
//#include <rev/SparkMaxConfig.h>
//#include <rev/SparkMaxConfigAccessor.h>
using namespace rev::spark;
#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/Kinematics/MecanumDriveKinematics.h>
#include <frc/MotorSafety.h>
#include <frc/drive/DifferentialDrive.h>

#include <frc/Encoder.h>
#include <frc/motorcontrol/PWMSparkMax.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 Robot() {
#if defined(__linux__) || defined(_WIN32)
    frc::CameraServer::StartAutomaticCapture();
#else
    std::fputs("Vision only available on Linux or Windows.\n", stderr);
    std::fflush(stderr);
#endif
  }


 private:
  //frc::SendableChooser<std::string> m_chooser;
  //const std::string kAutoNameDefault = "Default";
  //const std::string kAutoNameCustom = "My Auto";
  //std::string m_autoSelected;

// Gyro calibration constant, may need to be adjusted. Gyro value of 360 is
// set to correspond to one full revolution.
  private:static constexpr double kVoltsPerDegreePerSecond = 0.0128;


  static constexpr int kFrontLeftMotor = 6;
  static constexpr int kRearLeftMotor = 1;
  static constexpr int kFrontRightMotor = 2;
  static constexpr int kRearRightMotor = 3;

//uncomment the next 2 lines for Boom control
  static constexpr int kBoomChannel = 4;
  static constexpr int kGyroPort = 0;

 
 }; 
//CAN

  //using namespace rev::spark;

  //SparkMax m_max{6, SparkMax::MotorType::kBrushless};
  //SparkMax m_max{1, SparkMax::MotorType::kBrushless};
  //SparkMax m_max{2, SparkMax::MotorType::kBrushless};
  //SparkMax m_max{3, SparkMax::MotorType::kBrushless};

//PWM
static const int frontleftPwmChannel = 6, rearrightPwmChannel = 3, rearleftPwnChannel = 1, frontrightPwmChannel = 2;

frc::PWMSparkMax m_frontleftMotor{frontleftPwmChannel};
frc::PWMSparkMax m_rearrightMotor{rearrightPwmChannel};
frc::PWMSparkMax m_rearleftMotor{frontleftPwmChannel};
frc::PWMSparkMax m_frontrightMotor{frontrightPwmChannel};

SparkMax m_frontleftMotor{6, SparkMax::MotorType::kBrushless};
SparkMax m_rearleftMotor{3, SparkMax::MotorType::kBrushless};
SparkMax m_frontrightMotor{1, SparkMax::MotorType::kBrushless};
SparkMax m_rearrightMotor{2, SparkMax::MotorType::kBrushless};


  frc::MecanumDriveKinematics  m_rkinematics{m_frontLeftLocation, m_rearLeftLocation, m_frontRightLocation, m_rearRightLocation};
  frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
  frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
  frc::Translation2d m_rearLeftLocation{-0.381_m, 0.381_m};
  frc::Translation2d m_rearRightLocation{-0.381_m, -0.381_m};

//static WheelSpeeds frc::MecanumDriveKinematics::DrivePolar (double  xSpeed, double  ySpeed, double  zRotation, Rotation2dgyroAngle = 0_rad );

frc::MecanumDrive m_robotDrive{m_frontleftMotor, m_rearleftMotor, m_frontrightMotor, m_rearrightMotor};
//uncomment the next line for Boom control
rev::spark::SparkMax m_Boom(4, SparkMax::MotorType::kBrushless);

//frc::MotorSafety::MotorSafety	(		);	
//frc::MotorSafety::SetExpiration	(	units::second_t	expirationTime	);	


//uncomment the next 5 lines for Boom control
//rev::SparkRelativeEncoder m_encoder = m_Boom.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096);
//rev::SparkMaxClosedLoopController m_ClosedLoopController = m_Boom.GetClosedLoopController();

//double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
//m_Boom.Restore FactoryDefaults();

//m_ClosedLoopController.SetFeedbackDevice(m_encoder);

  int it = 0;
  int ia = 0;

  frc::XboxController m_driverController{0};

  frc::Timer         m_timer;

  

  bool leftbumperbutton = false;
  bool rightbumperbutton = false;

  double lefttriggeraxis = 0.0;
  double righttriggeraxis = 0.0;

  bool bA = false;
  bool bB = false;
  bool bX = false;
  bool bY = false;
  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;

  int ai_raw  = 0;
  double ai_voltage = 0.0;

  int ultra_raw = 0;
  double currentDistanceCentimeters = 0.0;
  double currentDistanceInches = 0.0;

  double xSpeed = 0.0;
  double ySpeed = 0.0;
  double zRotation = 0.0;

  double kOff = 0.0;
  double kReverse = 0.0;
  double kForward = 0.0;

  //double kGyroPort = 0.0;
  double voltage_scale_factor = 0.0;

  //AHRS *ahrs;
  //float yaw;
  

  //int kSize640x480 = 0;


  

  //int kSize640x480 = 0;

