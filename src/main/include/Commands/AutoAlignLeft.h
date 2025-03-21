// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoAlignLeft
    : public frc2::CommandHelper<frc2::Command, AutoAlignLeft> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
  AutoAlignLeft(DriveSubsystem *drivebase);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  double DesiredX = -0.44;
  double DesiredY = -0.18;
  double DesiredRot = -1.025;

  bool readyToExit = false;

  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::PIDController m_rotController;

  DriveSubsystem *m_drivebase;
};
