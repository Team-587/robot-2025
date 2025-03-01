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

using namespace rev::spark;




class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void allowClimb();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  #ifdef haveClimber
  // declared private and exposed only through public methods.
  SparkMax m_climberMotor{DriveConstants::kClimberCanId, SparkMax::MotorType::kBrushless};
  SparkMax m_climberMotor2{DriveConstants::kClimberCanId2, SparkMax::MotorType::kBrushless};
  SparkMax m_climberHopperMotor{DriveConstants::kClimberHopperCanId, SparkMax::MotorType::kBrushless};
  SparkMax m_climberHopperMotor2{DriveConstants::kClimberHopperCanId2, SparkMax::MotorType::kBrushless};
  #endif

  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};

  bool canClimb = false;
  
};
