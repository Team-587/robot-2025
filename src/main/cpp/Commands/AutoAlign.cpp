// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/AutoAlign.h"
#include <frc/geometry/Pose2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#ifdef usingAutoAlign
AutoAlign::AutoAlign(bool isRightScore, DriveSubsystem *drivebase)
  : m_xController(0.4,0.0,0), 
    m_yController(0.8,0.0,0), 
    m_rotController(0.008,0.0,0), 
    m_isRightScore(isRightScore), 
    m_drivebase(drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->AddRequirements(m_drivebase);
  std::cout << "Auto Align Constructer\n";
  Count = 20;
}
// Called when the command is initially scheduled.
void AutoAlign::Initialize() {
  //m_stopTimer.Start();
  //m_dontSeeTagTimer.Start();
  frc::SmartDashboard::PutData("PID X", &m_xController);
  frc::SmartDashboard::PutData("PID Y", &m_yController);
  frc::SmartDashboard::PutData("PID R", &m_rotController);
  if(m_isRightScore){
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  }else{
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");
  }
  double id = table->GetNumber("tid", 0.0);
  std::cout << "Auto Align Started: ID" << id << "\n";
  //RIGHT
  if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
    if(m_isRightScore){
      readyToExit = false;
      //DesiredX = 3.21;
      DesiredX = -0.47;
      //DesiredY = 3.88;
      DesiredY = 0.19;
      DesiredRot = -2.5;
    }else{
      readyToExit = false;
      DesiredX = -0.47;
      DesiredRot = -1.025;
      DesiredY = -0.18;
    }
  }else{
    readyToExit = true;
  }

  m_rotController.SetSetpoint(DesiredRot);
  m_rotController.SetTolerance(0.05);

  m_xController.SetSetpoint(DesiredX);
  m_xController.SetTolerance(0.05);

  m_yController.SetSetpoint(DesiredY);
  m_yController.SetTolerance(0.05);

  frc::SmartDashboard::PutNumber("ID", id);
  frc::SmartDashboard::PutNumber("Desired X", DesiredX);
  frc::SmartDashboard::PutNumber("Desired Y", DesiredY);
  frc::SmartDashboard::PutNumber("Desired Rot", DesiredRot);
  std::cout << "Auto Align Started: ID" << id << " " << readyToExit << "\n";
}

// Called repeatedly when this Command is scheduled to run
void AutoAlign::Execute() {
  //std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  double id = table->GetNumber("tid", 0.0);
  std::cout << "Auto Align Executed: ID" << id << "\n";
  std::vector<double> positions;
  if(m_isRightScore){
    positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");
  }else{
    positions = LimelightHelpers::getBotpose_TargetSpace("limelight-right");
  }

  if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
    if(positions[2] < -1.0){
      m_xController.SetP(0.2);
      m_yController.SetP(0.4);
      m_rotController.SetP(0.006);
    }else{
      m_xController.SetP(0.5);
      m_yController.SetP(1.0);
      m_rotController.SetP(0.008);
    }
    double xSpeed = m_xController.Calculate(positions[2]);
    double ySpeed = -m_yController.Calculate(positions[0]);
    double rotValue = -m_rotController.Calculate(positions[4]);

    m_drivebase->Drive((units::velocity::meters_per_second_t)xSpeed, (units::velocity::meters_per_second_t)ySpeed, (units::angular_velocity::radians_per_second_t)rotValue, false);
  }else{
    /*Count--;
    if(Count <= 0){
      readyToExit = true;
    }
    m_drivebase->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);*/
  }
  //}
  std::cout << "Auto Align Executed: ID" << id << " " << readyToExit << "\n";
}

// Called once the command ends or is interrupted.
void AutoAlign::End(bool interrupted) {
  std::cout << "Auto Align End\n";
  m_drivebase->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);

}

// Returns true when the command should end.
bool AutoAlign::IsFinished() {
  /*frc::Pose2d position = m_drivebase->m_poseEstimator.GetEstimatedPosition();
  double positionXError = (double)position.X() - DesiredX;
  double positionYError = (double)position.Y() - DesiredY;
  double positionRotError = (double)position.Rotation().Radians() - DesiredRot;*/

  if(readyToExit){
    std::cout << "Auto Align Finished 1\n";
    return true;
  }
  std::vector<double> positions;
  if(m_isRightScore){
    positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");
  }else{
    positions = LimelightHelpers::getBotpose_TargetSpace("limelight-right");
  }
  double positionXError = positions[2] - DesiredX;
  double positionYError = positions[0] - DesiredY;
  double positionRotError = positions[4] - DesiredRot;
  std::cout << "X Error" << positionXError << "\n";
  std::cout << "Y Error" << positionYError << "\n";
  std::cout << "Rot Error" << positionRotError << "\n";

  if((positionXError < 0.1 && positionXError > -0.1) && 
        (positionYError < 0.1 && positionYError > -0.1) && 
        (positionRotError < 1.0 && positionRotError > -1.0)){
    return true;
    std::cout << "Auto Align Finished 2\n";
  }else{
    return false;
  }
}
#endif