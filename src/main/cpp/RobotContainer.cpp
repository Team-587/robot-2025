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

RobotContainer::RobotContainer() : m_coralSubsystem(), m_lights(), m_drive(), m_autoAlignRight(true, &m_drive), m_autoAlignLeft(false, &m_drive){
  // Initialize all of your commands and subsystems here
  m_lights.setCoralSubsystem(&m_coralSubsystem);
  m_drive.setCoralDriveSubsystem(&m_coralSubsystem);
  // Configure the button bindings
  ConfigureButtonBindings();

  pathplanner::NamedCommands::registerCommand("Level 1", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL1); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL1Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 2", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL2); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(2.5_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL2Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 3", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL3); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(3.3_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL3Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(1.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Level 4", frc2::cmd::Sequence( 
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem}),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::LEVEL4); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(3.0_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseL4Speed); }, {&m_coralSubsystem}),
                                                         frc2::cmd::Wait(0.5_s),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}),
                                                         frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));

  pathplanner::NamedCommands::registerCommand("Intake", frc2::cmd::Sequence(
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}), 
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kHouseIntakeSpeed); }, {&m_coralSubsystem}),
                                                        frc2::cmd::Wait(1.0_s),
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setSpeed(CoralConstants::kBackspin); }, {&m_coralSubsystem})));
  
  pathplanner::NamedCommands::registerCommand("Stow",
                                                        frc2::cmd::RunOnce([this] {this->m_coralSubsystem.setState(CoralSubsystem::STOW); }, {&m_coralSubsystem}));

 pathplanner::NamedCommands::registerCommand("Auto Align", std::make_unique<AutoAlign>(true, &m_drive));

 pathplanner::NamedCommands::registerCommand("Tag 20", frc2::cmd::RunOnce([this] {this->m_drive.setPreferedAprilTag(20); }, {&m_drive}));
  pathplanner::NamedCommands::registerCommand("Tag 19", frc2::cmd::RunOnce([this] {this->m_drive.setPreferedAprilTag(19); }, {&m_drive}));


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
const std::string Curry_Str = "Curry";
const std::string LeBron_Str = "LeBron";
const std::string Level1_Str = "Level1";
const std::string SteveNash_Str = "Steve Nash";
const std::string AutoAlign_Str = "Auto Align";

Test = PathPlannerAuto(Test_Str).ToPtr().Unwrap();
Curry = PathPlannerAuto(Curry_Str).ToPtr().Unwrap();
LeBron = PathPlannerAuto(LeBron_Str).ToPtr().Unwrap();
Level1 = PathPlannerAuto(Level1_Str).ToPtr().Unwrap();
SteveNash = PathPlannerAuto(SteveNash_Str).ToPtr().Unwrap();
//Autoalign = PathPlannerAuto(AutoAlign_Str).ToPtr().Unwrap();

/*static auto segGroup = std::make_shared<frc2::SequentialCommandGroup>(
    m_autoAlignRight,
    PathPlannerAuto(AutoAlign_Str)
);*/

m_chooser.SetDefaultOption(Test_Str, Test.get());
m_chooser.AddOption(Curry_Str, Curry.get());
m_chooser.AddOption(LeBron_Str, LeBron.get());
m_chooser.AddOption(Level1_Str, Level1.get());
m_chooser.AddOption(SteveNash_Str, SteveNash.get());
//m_chooser.AddOption(AutoAlign_Str, Autoalign.get());
//m_chooser.AddOption("ABC", segGroup.get());

frc::SmartDashboard::PutData("Auto", &m_chooser);
std::cout << "Autos Complete\n";

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

  frc2::JoystickButton startButtonCoDrive{&m_codriverController, frc::XboxController::Button::kStart};
  startButtonCoDrive.OnTrue(&m_StartClimb);

  frc2::JoystickButton backButtonLeft{&m_codriverController, frc::XboxController::Button::kLeftStick};
  backButtonLeft.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::ALGAE1); }, {&m_coralSubsystem}));

  frc2::JoystickButton backButtonRight{&m_codriverController, frc::XboxController::Button::kRightStick};
  backButtonRight.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::ALGAE2); }, {&m_coralSubsystem}));

  frc2::JoystickButton codriverRB{&m_codriverController, frc::XboxController::Button::kRightBumper};
  codriverRB.OnTrue(new frc2::RunCommand([this] { m_coralSubsystem.setState(CoralSubsystem::ALGAESCORE); }, {&m_coralSubsystem}));

  frc2::JoystickButton backButtonRightDrive{&m_driverController, frc::XboxController::Button::kRightStick};
  backButtonRightDrive.WhileTrue(&m_autoAlignRight);

  frc2::JoystickButton backButtonLeftDrive{&m_driverController, frc::XboxController::Button::kLeftStick};
  backButtonLeftDrive.WhileTrue(&m_autoAlignLeft);

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

