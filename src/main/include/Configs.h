#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

using namespace rev::spark;

namespace Configs {
class MAXSwerveModule {
 public:
  static SparkMaxConfig& DrivingConfig() {
    static SparkMaxConfig drivingConfig{};

    // Use module constants to calculate conversion factors and feed forward
    // gain.
    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;
    double drivingVelocityFeedForward =
        1 / ModuleConstants::kDriveWheelFreeSpeedRps;

    drivingConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(50);
    drivingConfig.encoder
        .PositionConversionFactor(drivingFactor)          // meters
        .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    drivingConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(0.04, 0, 0)
        .VelocityFF(drivingVelocityFeedForward)
        .OutputRange(-1, 1);

    return drivingConfig;
  }

  static SparkMaxConfig& TurningConfig() {
    static SparkMaxConfig turningConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    turningConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    turningConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    turningConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return turningConfig;
  }
};

static SparkMaxConfig& wristConfig() {
    static SparkMaxConfig wristConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    wristConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    wristConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    wristConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return wristConfig;
  }

  static SparkMaxConfig& uppiesConfig1() {
    static SparkMaxConfig uppiesConfig1{};

    // Use module constants to calculate conversion factor
    double turningFactor = 1.432 * std::numbers::pi;

    uppiesConfig1.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);
    uppiesConfig1
        .alternateEncoder
        .CountsPerRevolution(8192)
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    uppiesConfig1.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return uppiesConfig1;
  }

  static SparkMaxConfig& uppiesConfig2() {
    static SparkMaxConfig uppiesConfig2{};

    // Use module constants to calculate conversion factor
    //double turningFactor = 2 * std::numbers::pi;

    uppiesConfig2.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(40);
    /*uppiesConfig2
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    uppiesConfig2.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);*/

    uppiesConfig2.Follow(DriveConstants::kUppies1CanId, true);

    return uppiesConfig2;
  }

  static SparkMaxConfig& ballWristConfig() {
    static SparkMaxConfig ballWristConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    ballWristConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(20);
    ballWristConfig
        .absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(false)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    ballWristConfig.closedLoop
        .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        // These are example gains you may need to them for your own robot!
        .Pid(1, 0, 0)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    return ballWristConfig;
  }
}  // namespace Configs
