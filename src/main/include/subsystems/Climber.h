// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <iostream>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include "Constants.h"
#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include <frc/AnalogInput.h>
#include <frc/Servo.h>


using namespace rev::spark;




class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void allowClimb();

  void dropHopper();

  void xPressed();

  void climberIn();
  
  void climberOut();

  bool climbMode();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  //#ifdef haveClimber
  // declared private and exposed only through public methods.
  SparkMax m_climberMotor{DriveConstants::kClimberCanId, SparkMax::MotorType::kBrushless};
  frc::Servo m_hopperServo{2};
  frc::Servo m_climberServo{1};
  //#endif

  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};

  bool canClimb = false;
  bool xIsPressed = false;
  
};
