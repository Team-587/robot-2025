// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/SparkFlex.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/XboxController.h>
#include <thread>

#include "Constants.h"
#include "Configs.h"

using namespace rev::spark;

class BallSubsystem : public frc2::SubsystemBase {
 public:
  BallSubsystem();

  enum ballStates {
    INTAKE,
    STOW,
    HALFSTOW,
    SCORE
  };

ballStates DesiredState{STOW};
ballStates CurrentState = STOW;


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setState(ballStates newState);
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  #ifndef usingNeo
  SparkFlex m_ballMotor{DriveConstants::kBallPickupIntakeCanId, SparkMax::MotorType::kBrushless};
  SparkMax m_ballWristMotor{DriveConstants::kBallPickupWristCanId, SparkMax::MotorType::kBrushless};

  SparkAbsoluteEncoder m_ballAbsoluteEncoder = m_ballWristMotor.GetAbsoluteEncoder();

  SparkClosedLoopController m_ballClosedLoopController = m_ballWristMotor.GetClosedLoopController();
  #endif
  
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};
};
