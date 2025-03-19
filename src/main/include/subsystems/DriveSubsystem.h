// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include "studica/AHRS.h"
#include "frc/XboxController.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/CoralSubsystem.h"
//#include <choreo/trajectory/SwerveSample.h>
//#include <frc/controller/PIDController.h>

#include "Constants.h"
#include "MAXSwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  //void FollowTrajectory(const choreo::SwerveSample& sample);

  //void AutonomousInit();

  //void AutonomousPeriodic();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * 
   * 
   */

  void setCoralDriveSubsystem(CoralSubsystem *tmpcs){p_coralSubsystem = tmpcs;}; 

  void setPreferedAprilTag(int tag);
  
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  inline frc::ChassisSpeeds getSpeeds() {
      return kDriveKinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()});
    }

  void driveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds);

  double id;
  int preferedTag = -1;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  //std::optional<choreo::Trajectory<choreo::SwerveSample>> trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Score");

  

  double distanceAprilTag;
  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double distanceOff;

  //frc::PIDController xController{10.0, 0.0, 0.0};
  //frc::PIDController yController{10.0, 0.0, 0.0};
  //frc::PIDController headingController{7.5, 0.0, 0.0};


  // The gyro sensor
  //frc::ADIS16470_IMU m_gyro;
  studica::AHRS m_NavX{studica::AHRS::NavXComType::kMXP_SPI};
  //studica::AHRS m_NavX{studica::AHRS::NavXComType::kUSB1};
  // Odometry class for tracking robot pose
  // 4 defines the number of modules

  frc::Field2d m_field;

  frc::SwerveDriveOdometry<4> m_odometry;

  CoralSubsystem *p_coralSubsystem = NULL;

  public: 
  //frc::SwerveDrivePoseEstimator<4> m_poseEstimator;
};
