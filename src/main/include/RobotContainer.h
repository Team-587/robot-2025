// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/CoralSubsystem.h"
#include "subsystems/BallSubsystem.h"
#include "subsystems/Lights.h"
#include "subsystems/Climber.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  CoralSubsystem m_coralSubsystem;
  BallSubsystem m_ballSubsystem;
  Lights m_lights;

private:

  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...
 
  // The robot's subsystems
  DriveSubsystem m_drive;
  Climber m_climber;
  
  

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  frc2::InstantCommand m_StartClimb{[this] {m_climber.allowClimb();}, {&m_climber}};

  
  std::unique_ptr<frc2::Command> Test;
  std::unique_ptr<frc2::Command> ML4_Right;
  std::unique_ptr<frc2::Command> ML3_Left;
  std::unique_ptr<frc2::Command> Level1;
  std::unique_ptr<frc2::Command> MiddleLevel3;
};
