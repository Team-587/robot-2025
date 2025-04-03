// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/AutoAlignRight.h"
#include <frc/geometry/Pose2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

AutoAlignRight::AutoAlignRight(DriveSubsystem *drivebase)
  : m_xController(0.4,0.0,0), 
    m_yController(0.8,0.0,0), 
    m_rotController(0.01,0.0,0), 
    m_drivebase(drivebase) {
  // Use addRequirements() here to declare subsystem dependencies.
  this->AddRequirements(m_drivebase);
  std::cout << "RIGHT\n";
}

// Called when the command is initially scheduled.
void AutoAlignRight::Initialize() {
  /*frc::SmartDashboard::PutData("PID X", &m_xController);
  frc::SmartDashboard::PutData("PID Y", &m_yController);
  frc::SmartDashboard::PutData("PID R", &m_rotController);*/

  std::shared_ptr<nt::NetworkTable> tableRight = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  double id = tableRight->GetNumber("tid", 0.0);

  if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
    readyToExit = false;
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
}

// Called repeatedly when this Command is scheduled to run
void AutoAlignRight::Execute() {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");
  double id = table->GetNumber("tid", 0.0);
  std::vector<double> positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");

  if((id >= 6 && id <= 11) || (id >= 17 && id <= 22)){
    if(positions[2] < -1.0){
      m_xController.SetP(0.4);
      m_yController.SetP(0.8);
      m_rotController.SetP(0.012);
    }else{
      m_xController.SetP(0.8);
      m_yController.SetP(1.0);
      m_rotController.SetP(0.008);
    }
    double xSpeed = m_xController.Calculate(positions[2]);
    double ySpeed = -m_yController.Calculate(positions[0]);
    double rotValue = -m_rotController.Calculate(positions[4]);

    m_drivebase->Drive((units::velocity::meters_per_second_t)xSpeed, (units::velocity::meters_per_second_t)ySpeed, (units::angular_velocity::radians_per_second_t)rotValue, false);
  }
}

// Called once the command ends or is interrupted.
void AutoAlignRight::End(bool interrupted) {
  m_drivebase->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
}

// Returns true when the command should end.
bool AutoAlignRight::IsFinished() {
  if(readyToExit){
    std::cout << "Auto Align Finished 1\n";
    return true;
  }

  std::vector<double> positions = LimelightHelpers::getBotpose_TargetSpace("limelight-left");
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
