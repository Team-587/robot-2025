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
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <cmath>
#include <vector>
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "subsystems/CoralSubsystem.h"
#include <photon/PhotonUtils.h>
//#include <choreo/trajectory/Trajectory.h>
//#include <choreo/Choreo.h>

using namespace DriveConstants;
using namespace pathplanner;

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
                     m_NavX.GetRotation2d().Radians()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}/*,
      m_poseEstimator{kDriveKinematics, frc::Rotation2d(units::radian_t{
                     m_NavX.GetRotation2d().Radians()}),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                    frc::Pose2d{}, {0.1, 0.1, 0.1}, {0.1, 0.1, 0.1}}*/

{

                  //auto trajectory = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Score");

                  //headingController.EnableContinuousInput(-M_PI, M_PI);

                  

                  frc::SmartDashboard::PutData("Field", &m_field);

                  RobotConfig config = RobotConfig::fromGUISettings();

                  AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
       std::make_shared<PPHolonomicDriveController>( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            pathplanner::PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(7.0, 0.0, 0.0) ),
            config, // Rotation PID constants
             //pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance =frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //m_field.SetRobotPose(m_odometry.GetPose());
  
  //Telemetry
  frc::SmartDashboard::PutNumber("Yaw", m_NavX.GetYaw());
  frc::SmartDashboard::PutNumber("Pitch", m_NavX.GetPitch());
  frc::SmartDashboard::PutNumber("Roll", m_NavX.GetRoll());

  frc::SmartDashboard::PutNumber("Pigeon Yaw", pigeon.GetYaw().GetValueAsDouble());
  
  //std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  //auto keys = 
  //for(const std::string& key : keys) {

  //}

  /*std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  //double targetArea = table->GetNumber("ta", 0.0);
  //double targetSkew = table->GetNumber("ts", 0.0);

  frc::SmartDashboard::PutNumber("LL tx", targetOffsetAngle_Horizontal);
  frc::SmartDashboard::PutNumber("LL ty", targetOffsetAngle_Vertical);*/
    
  //std::vector<double> positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");
  /*if(positions.size() >= 5){
  frc::SmartDashboard::PutNumber("Limelight Left X", positions[2]);
  frc::SmartDashboard::PutNumber("Limelight Left Y", positions[0]);
  frc::SmartDashboard::PutNumber("Limelight Left Rot", positions[4]);
  }*/
  //std::vector<double> positions = LimelightHelpers::getBotpose_TargetSpace("limelight-right");
  //frc::SmartDashboard::PutNumber("LL ta", targetArea);
  //frc::SmartDashboard::PutNumber("LL ts", targetSkew);

  /*double cameraDegree = 30.0;
  double cameraHeight = 6.0;
  double aprilTagHeight = 12.25;

  distanceAprilTag = (aprilTagHeight - cameraHeight)/tan((targetOffsetAngle_Vertical + cameraDegree) * std::numbers::pi / 180.0);
  frc::SmartDashboard::PutNumber("Distance April Tag", distanceAprilTag);
  distanceOff = (distanceAprilTag - 70.0) / 39.37;
  frc::SmartDashboard::PutNumber("Distance Off", distanceOff);
  double distanceCenterAprilTag = distanceAprilTag * (tan((targetOffsetAngle_Horizontal) * std::numbers::pi / 180));
  frc::SmartDashboard::PutNumber("Distance From Center", distanceCenterAprilTag);*/

  m_odometry.Update(frc::Rotation2d(units::radian_t{
                        m_NavX.GetRotation2d().Radians()}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  /*m_poseEstimator.Update(frc::Rotation2d(units::radian_t{
                        m_NavX.GetRotation2d().Radians()}),
                        {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                        m_frontRight.GetPosition(), m_rearRight.GetPosition()});
    bool doRejectUpdate = false;
    LimelightHelpers::SetRobotOrientation("limelight", (double)m_poseEstimator.GetEstimatedPosition().Rotation().Degrees(), 0, 0, 0, 0, 0);
        LimelightHelpers::PoseEstimate mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(std::abs(m_NavX.GetRate()) > 720) {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate) {
            m_poseEstimator.SetVisionMeasurementStdDevs({0.7, 0.7, 9999999});
            m_poseEstimator.AddVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
        }

        frc::Pose2d position = m_poseEstimator.GetEstimatedPosition();
        id = table->GetNumber("tid", 0.0);
        //frc::SmartDashboard::PutNumber("ID", id);
        frc::SmartDashboard::PutNumber("position X", (double)position.X());
        frc::SmartDashboard::PutNumber("position Y", (double)position.Y());

    if(frc::DriverStation::IsAutonomousEnabled()){
        std::cout << "position" << (double)m_poseEstimator.GetEstimatedPosition().X() << ", " << (double)m_poseEstimator.GetEstimatedPosition().Y() << "\n";
    } 
    */      
}
  

void DriveSubsystem::driveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds){
    frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, 0.02_s);
    double distanceCheck = -0.9;
    double pidD = 0.0003;
    //targetSpeeds.vx = -targetSpeeds.vx;
   // targetSpeeds.vy = -targetSpeeds.vy;
   // targetSpeeds.omega = -targetSpeeds.omega;
    
   if(intake) {
    std::cout << "Going To Intake\n";
    auto results = photonCam.GetAllUnreadResults();
    units::degree_t targetYaw = 0.0_deg;
    units::meter_t targetRange = 0.0_m;
    if(results.size() > 0) {
        std::cout << "result\n";
        auto result = results[results.size() - 1];
        if(result.HasTargets()) {
            for(auto& target : result.GetTargets()) {
                if(target.GetFiducialId() == 12 || target.GetFiducialId() == 13 || target.GetFiducialId() == 1 || target.GetFiducialId() == 2) {
                    autoAlignHPLight = true;
                    targetYaw = units::degree_t{target.GetYaw()};
                    targetRange = photon::PhotonUtils::CalculateDistanceToTarget(
                        0.9398_m,
                        1.4859_m,
                        57_deg,
                        units::degree_t{target.GetPitch()});
                        std::cout << "m\n";
                        double xDist = (double)targetRange * std::cos((double)targetYaw * (PI / 180.0));
                        double yDist = (double)targetRange * std::sin((double)targetYaw * (PI / 180.0));
                        std::cout << "a\n";
                        frc::PIDController m_xController(9.0, 0.0, 0.0);
                        frc::PIDController m_yController(5.0, 0.0, 0.0);
                        frc::PIDController m_rotController(0.03, 0.0, 0.0);

                        m_rotController.SetSetpoint(0);
                        m_rotController.SetTolerance(0.05);
                        m_xController.SetSetpoint(0.19);
                        m_xController.SetTolerance(0.01);
                        m_yController.SetSetpoint(0);
                        m_yController.SetTolerance(0.05);

                        double xSpeed = m_xController.Calculate(xDist);
                        double ySpeed = -m_yController.Calculate(yDist);
                        double rotValue = -m_rotController.Calculate((double)targetYaw);

                        targetSpeeds.vx = (units::velocity::meters_per_second_t)xSpeed;
                        targetSpeeds.vy = (units::velocity::meters_per_second_t)ySpeed;
                        targetSpeeds.omega = (units::angular_velocity::radians_per_second_t)rotValue; 
                        std::cout<<"ligning up " << xSpeed << " "<< ySpeed << " " << rotValue << " " << (double)targetRange << " " << (double)targetYaw << " " << xDist << " " << yDist << "\n";
                        
                }
                else{
                    autoAlignHPLight = false;
                }
            }
        }
    }
   } else {
   if(Right){
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
    id = table->GetNumber("tid", 0.0);
   }else{
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");
    id = table->GetNumber("tid", 0.0);
   }
   if(p_coralSubsystem->haveCoral){
        if(id == preferedTagRed || id == preferedTagBlue){
            if(Right){
                //std::cout << Right << " Right\n";
                table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
                id = table->GetNumber("tid", 0.0);
                //std::cout << "Auto Align Executed: ID" << id << "\n";
                positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");
                DesiredX = -0.45;
                DesiredY = 0.18;
                DesiredRot = -2.5;
                distanceCheck = -1.2;
                pidD = 0.0003;
                //distanceCheck = -1.3;
            }else if(Right == false){
                table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");
                id = table->GetNumber("tid", 0.0);
                //std::cout << "Auto Align Executed: ID" << id << "\n";
                positions = LimelightHelpers::getBotpose_TargetSpace("limelight-right");
                DesiredX = -0.45;
                DesiredY = -0.18;
                DesiredRot = -1.025;
                distanceCheck = -1.2;
                pidD = 0.0;
            }
            if(positions[2] > distanceCheck){
                //frc::PIDController m_xController(1.2, 0.0, 0.0);

                autoAlignReefLight = true;

                frc::PIDController m_xController(2.0, 0.0, 0.0);
                frc::PIDController m_yController(2.0, 0.0, pidD);
                frc::PIDController m_rotController(0.03, 0.0, 0.0);

                m_rotController.SetSetpoint(DesiredRot);
                m_rotController.SetTolerance(0.05);
                m_xController.SetSetpoint(DesiredX);
                m_xController.SetTolerance(0.05);
                m_yController.SetSetpoint(DesiredY);
                m_yController.SetTolerance(0.05);

                double xSpeed = m_xController.Calculate(positions[2]);
                double ySpeed = -m_yController.Calculate(positions[0]);
                double rotValue = -m_rotController.Calculate(positions[4]);

                targetSpeeds.vx = (units::velocity::meters_per_second_t)xSpeed;
                targetSpeeds.vy = (units::velocity::meters_per_second_t)ySpeed;
                targetSpeeds.omega = (units::angular_velocity::radians_per_second_t)rotValue;
                std::cout << "See April Tag: " << id;
            }else{
                autoAlignReefLight = false;
            }
        
        }
   }
 }
    // if (frc::DriverStation::IsAutonomousEnabled()) {
    //     std::cout << "auto drive speed x:" << (double)targetSpeeds.vx << " y:" << (double)targetSpeeds.vy << " ang:" << (double)targetSpeeds.omega << "\n";
    // }
    
    auto targetStates = kDriveKinematics.ToSwerveModuleStates(targetSpeeds);
    SetModuleStates(targetStates);

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {


//ySpeed = 0.0_mps;
//rot = 0.0_rad_per_s;

frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
double rightTriggerValue = (m_driverController.GetRightTriggerAxis() * -.8) + 1.0;
  // Convert the commanded speeds into the correct units for the drivetrain

  if(xSpeed < -0.01_mps && xSpeed > -0.1_mps){
        xSpeed = 0.0_mps;
    }
  if(ySpeed < 0.1_mps && ySpeed > 0.01_mps){
        ySpeed = 0.0_mps;
    }

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

void DriveSubsystem::ZeroHeading() { m_NavX.Reset(); ResetEncoders(); }

double DriveSubsystem::GetTurnRate() {
  //return -m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
  //GetRate might not be the right method to call
  return -m_NavX.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() { 
    return m_odometry.GetPose();
    //return m_poseEstimator.GetEstimatedPosition(); 
}

/*void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample& sample) {
    frc::Pose2d pose = GetPose();


    units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
    units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};
    units::radians_per_second_t headingFeedback{headingController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())};

    frc::ChassisSpeeds::FromFieldRelativeSpeeds(sample.vx + xFeedback, sample.vy + yFeedback, sample.omega + headingFeedback, DriveSubsystem::GetHeading());
}
*/
void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
    //frc::Pose2d tmpPose = GetPose();
    //std::cout << "Reset Odometry start X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Degrees() << "\n";
    //std::cout << "Reset Odometry set   X:" << (double)pose.X() << " Y:" << (double)pose.Y() << " Rot:" << (double)pose.Rotation().Degrees() << "\n";
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
    //tmpPose = GetPose();
    //std::cout << "Reset Odometry after X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Degrees() << "\n";
} 

void DriveSubsystem::setPreferedAprilTag(int tag){
    preferedTagBlue = tag;
    if(tag == 22){
        preferedTagRed = 9;
    }else if(tag == 21){
        preferedTagRed = 10;
    }else if(tag == 20){
        preferedTagRed = 11;
    }else if(tag == 19){
        preferedTagRed = 6;
    }else if(tag == 18){
        preferedTagRed = 7;
    }else if(tag == 17){
        preferedTagRed = 8;
    }
}

void DriveSubsystem::setRight(bool right){
    Right = right;
}

void DriveSubsystem::coralCheck() {}

void DriveSubsystem::setIntaking(bool intaking) {
    intake = intaking;
    std::cout << "setting Intake: " << intake << " \n";
}