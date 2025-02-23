// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using namespace pathplanner;

RobotContainer::RobotContainer() : m_coralSubsystem(), m_lights() {
  // Initialize all of your commands and subsystems here
  m_lights.setCoralSubsystem(&m_coralSubsystem);
  // Configure the button bindings
  ConfigureButtonBindings();

  pathplanner::NamedCommands::registerCommand("Level 1", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL1); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL1Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 2", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL2); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(2.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL2Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 3", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL3); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(2.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL3Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 4", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL4); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(2.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL4Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem})));

  pathplanner::NamedCommands::registerCommand("Intake", frc2::cmd::Sequence(
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}), 
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseIntakeSpeed); }, {&m_coralSubsystem}),
                                                        frc2::cmd::Wait(1.5_s),
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true);
      },
      {&m_drive}));

const std::string Test_Str = "Test";
const std::string Test2_Str = "Test 2";
const std::string ML4_Left_Str = "ML4 - Left";
const std::string ML4_Right_Str = "ML4 - Right";

Test = PathPlannerAuto(Test_Str).ToPtr().Unwrap();
Test_2 = PathPlannerAuto(Test2_Str).ToPtr().Unwrap();
ML4_Left = PathPlannerAuto(ML4_Left_Str).ToPtr().Unwrap();
ML4_Right = PathPlannerAuto(ML4_Right_Str).ToPtr().Unwrap();

m_chooser.SetDefaultOption(Test_Str, Test.get());
m_chooser.AddOption(Test2_Str, Test_2.get());
m_chooser.AddOption(ML4_Left_Str, ML4_Left.get());
m_chooser.AddOption(ML4_Right_Str, ML4_Right.get());
//m_chooser.AddOption(Test_Str, Test.get());

frc::SmartDashboard::PutData("Auto", &m_chooser);

}

void RobotContainer::ConfigureButtonBindings() {
  //frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
  
  frc2::JoystickButton startButton{&m_driverController, frc::XboxController::Button::kStart};
  startButton.OnTrue(&m_ZeroHeading);

  frc2::JoystickButton coDriverX(&m_codriverController, frc::XboxController::Button::kX);
  coDriverX.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::LEVEL1); }, {&m_coralSubsystem}));

  frc2::JoystickButton coDriverY(&m_codriverController, frc::XboxController::Button::kY);
  coDriverY.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::LEVEL2); }, {&m_coralSubsystem}));

  frc2::JoystickButton coDriverB(&m_codriverController, frc::XboxController::Button::kB);
  coDriverB.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::LEVEL3); }, {&m_coralSubsystem}));

  frc2::JoystickButton coDriverA(&m_codriverController, frc::XboxController::Button::kA);
  coDriverA.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::LEVEL4); }, {&m_coralSubsystem}));

 // frc2::JoystickButton coDriverRB(&m_codriverController, frc::XboxController::Button::kRightBumper);
  //coDriverRB.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::INTAKE); }, {&m_coralSubsystem}));

  frc2::JoystickButton coDriverLB(&m_codriverController, frc::XboxController::Button::kLeftBumper);
  coDriverLB.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}));

  //frc2::JoystickButton coDriverRB(&m_codriverController, frc::XboxController::Button::kRightBumper);
  //coDriverRB.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::SCORE); }, {&m_coralSubsystem}));

  frc2::JoystickButton DriveLB(&m_driverController, frc::XboxController::Button::kLeftBumper);
  DriveLB.WhileTrue(new frc2::RunCommand([this] {m_ballSubsystem.setState(BallSubsystem::SCORE); }, {&m_ballSubsystem}));

  frc2::JoystickButton DriveRB(&m_driverController, frc::XboxController::Button::kRightBumper);
  DriveRB.WhileTrue(new frc2::RunCommand([this] {m_ballSubsystem.setState(BallSubsystem::INTAKE); }, {&m_ballSubsystem}));

  DriveRB.WhileFalse(new frc2::RunCommand([this] {m_ballSubsystem.setState(BallSubsystem::STOW); }, {&m_ballSubsystem}));
  DriveLB.WhileFalse(new frc2::RunCommand([this] {m_ballSubsystem.setState(BallSubsystem::STOW); }, {&m_ballSubsystem}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
  // Set up config for trajectory
  /*frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));*/
}

