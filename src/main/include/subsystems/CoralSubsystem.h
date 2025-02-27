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

class CoralSubsystem : public frc2::SubsystemBase {
 public:
  CoralSubsystem();

  enum coralState {
    STOW,
    INTAKE,
    MOVEMENT,
    LEVEL1,
    LEVEL2,
    LEVEL3,
    LEVEL4,
    SCORE,
    ALGAE1,
    ALGAE2,
    ALGAESCORE

  };

  coralState desiredState{STOW};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void setState(coralState newState);
  void setSpeed(double speed);

  bool haveCoral;
  bool readyToScore;

  double autoSpeed;

 private:

  bool checkWristAngle(double wristAngle);

  bool checkUppiesHeight(double uppiesHeight);



#ifndef usingNeo
SparkFlex m_houseMotor{DriveConstants::kHouseCanId, SparkMax::MotorType::kBrushless};
SparkMax m_wristMotor{DriveConstants::kHouseWristCanId, SparkMax::MotorType::kBrushless};
SparkAbsoluteEncoder m_wristEncoder = m_wristMotor.GetAbsoluteEncoder();
SparkClosedLoopController m_wristClosedLoopController = m_wristMotor.GetClosedLoopController();
SparkMax m_uppiesMotor1{DriveConstants::kUppies1CanId, SparkMax::MotorType::kBrushless};
SparkMax m_uppiesMotor2{DriveConstants::kUppies2CanId, SparkMax::MotorType::kBrushless};


SparkMaxAlternateEncoder m_uppiesEncoder1 = m_uppiesMotor1.GetAlternateEncoder();

SparkClosedLoopController m_uppies1ClosedLoopController = m_uppiesMotor1.GetClosedLoopController();
#endif

frc::DigitalInput m_houseSwitch{DriveConstants::kHouseSwitch};
frc::DigitalInput m_uppiesSwitch{DriveConstants::kUppiesSwitch};

frc::AnalogInput m_distanceSensor{DriveConstants::kDistanceSensor};

frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};
 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
