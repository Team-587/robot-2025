// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "LimelightHelpers.h"
#include <frc/XboxController.h>
#include "RobotContainer.h"

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     //m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                     m_NavX.GetRotation2d().Radians()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  //Telemetry
  frc::SmartDashboard::PutNumber("Yaw", m_NavX.GetYaw());
  frc::SmartDashboard::PutNumber("Pitch", m_NavX.GetPitch());
  frc::SmartDashboard::PutNumber("Roll", m_NavX.GetRoll());

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  //double targetArea = table->GetNumber("ta", 0.0);
  //double targetSkew = table->GetNumber("ts", 0.0);

  frc::SmartDashboard::PutNumber("LL tx", targetOffsetAngle_Horizontal);
  frc::SmartDashboard::PutNumber("LL ty", targetOffsetAngle_Vertical);
  //frc::SmartDashboard::PutNumber("LL ta", targetArea);
  //frc::SmartDashboard::PutNumber("LL ts", targetSkew);

  distanceAprilTag = 20/tan(targetOffsetAngle_Vertical * std::numbers::pi / 180.0);
  frc::SmartDashboard::PutNumber("Distance April Tag", distanceAprilTag);
  distanceOff = (distanceAprilTag - 70.0) / 39.37;
  frc::SmartDashboard::PutNumber("Distance Off", distanceOff);

  m_odometry.Update(frc::Rotation2d(units::radian_t{
                        //m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
                        m_NavX.GetRotation2d().Radians()}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {

frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
double rightTriggerValue = (m_driverController.GetRightTriggerAxis() * -.8) + 1.0;
  // Convert the commanded speeds into the correct units for the drivetrain

  xSpeed = xSpeed * rightTriggerValue;
  ySpeed = ySpeed * rightTriggerValue;
  rot = rot * rightTriggerValue;
  
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;


  frc::SmartDashboard::PutNumber("xSpeed", (double)xSpeed);
  frc::SmartDashboard::PutNumber("ySpeed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("rot", (double)rot);

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    //m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
                    m_NavX.GetRotation2d().Radians()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {
  //return frc::Rotation2d(
             //units::radian_t{m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}).Degrees();
  return frc::Rotation2d(units::radian_t{m_NavX.GetRotation2d().Radians()}).Degrees();
}

void DriveSubsystem::ZeroHeading() { m_NavX.Reset(); }

double DriveSubsystem::GetTurnRate() {
  //return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
  //GetRate might not be the right method to call
  return -m_NavX.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
