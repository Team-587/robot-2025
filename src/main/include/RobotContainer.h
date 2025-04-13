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
#include "subsystems/Lights.h"
#include "subsystems/Climber.h"
//#include "Commands/AutoAlign.h"
#include "Commands/AutoAlignLeft.h"
#include "Commands/AutoAlignRight.h"

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
  Lights m_lights;
  DriveSubsystem m_drive;
  //AutoAlign m_autoAlignRight;
  //AutoAlign m_autoAlignLeft;
  AutoAlignLeft m_autoAlignLeft;
  AutoAlignRight m_autoAlignRight;

private:

  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...
 
  // The robot's subsystems
  Climber m_climber;
  
  

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
  
  frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  frc2::InstantCommand m_StartClimb{[this] {m_climber.allowClimb();}, {&m_climber}};
  frc2::InstantCommand m_lightsClimb{[this] {m_lights.climbMode();}, {&m_lights}};
  frc2::InstantCommand m_coralClimb{[this] {m_coralSubsystem.climbMode();}, {&m_coralSubsystem}};
  frc2::InstantCommand m_dropHopper{[this] {m_climber.dropHopper();}, {&m_climber}};
  frc2::InstantCommand m_servoMotor{[this] {m_climber.xPressed();}, {&m_climber}};
  frc2::InstantCommand m_algaeHigh{[this] {m_coralSubsystem.algaeRemoveHigh(); }, {&m_coralSubsystem}};
  frc2::InstantCommand m_algaeLow{[this] {m_coralSubsystem.algaeRemoveLow(); }, {&m_coralSubsystem}};
  frc2::InstantCommand m_processorScore{[this] {m_coralSubsystem.processorScore(); }, {&m_coralSubsystem}};
  frc2::InstantCommand m_processorHeight{[this] {m_coralSubsystem.processorHeight(); }, {&m_coralSubsystem}};

  
  std::unique_ptr<frc2::Command> Test;
  std::unique_ptr<frc2::Command> LeBron;
  std::unique_ptr<frc2::Command> Curry;
  std::unique_ptr<frc2::Command> Level1;
  std::unique_ptr<frc2::Command> SteveNash;

  //std::unique_ptr<frc2::Command> Iman_Shumpert_Left;
  std::unique_ptr<frc2::Command> Iman_Shumpert;
  //std::unique_ptr<frc2::Command> Lance_Stephenson_Left;
  std::unique_ptr<frc2::Command> Lance_Stephenson;
  //std::unique_ptr<frc2::Command> Autoalign;


};
  