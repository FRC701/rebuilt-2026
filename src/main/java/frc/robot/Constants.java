// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class UtilityConstants {
    /** Set to false before competition deploy to save bandwidth and loop time. */
    public static final boolean kTuningMode = false;
  }

  public static class OperatorConstants {
    public static final int kXboxControllerPort = 1;
    public static final int kPs4ControllerPort = 0;
  }

  // Agitator IDs : 20s
  public static class AgitatorConstants {
    public static final int kAgitatorLeftMotor = 21;
    public static final int kAgitatorRightMotor = 22;
    public static final double kAgitatorVoltIn = 7;
    public static final double kAgitatorVoltOut = -4;
    // Simulation
    public static final double kSimAgitatorGearRatio = 2; // also a planetary
    public static final double kSimAgitatorMOI = 0.001; // kg*m^2
  }

  // Feeder Motor Ids = 30s
  public static class FeederConstants {
    public static final int kFeederLeftMotor = 31;
    public static final int kFeederRightMotor = 32;
    // 3 is a placeholder for motor voltage
    public static final double kFeederVolt = 3;
    // Simulation
    public static final double kSimFeederGearRatio = 2; // also a planetary
    public static final double kSimFeederMOI = 0.0001; // kg*m^2
  }

  // Climber Motor Ids = 60s
  public static class ClimberConstants {
    // Leader is currently the left motor (facing the climber side)
    public static final int kClimberLeader = 62;
    public static final int kClimberFollower = 61;

    // The position/height of the climber measured with built in encoders
    public static final double kExtensionPosition = 7.4; // Inches
    public static final double kRetractPosition = 0.75; // Inches
    public static final double kLockPosition = 0;

    // PID Constants for Climbing
    public static final double kP = 0.1;
    public static final double kI = 0.1;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorArm = 11;
    public static final int kIntakeMotorRoller = 12;
    // The number of rotations using the falcon's encoder
    public static final double kExtensionPosition = 4.5; // 4.7
    public static final double kExtentionCycleUpPos = kExtensionPosition - 0.8;
    public static final double kRetractPosition = 0; // Intake is retract ed and in the bot

    // PID Constants for Intake Extension
    public static final double ExtendkP = 1.51337; // 1.51227
    public static final double ExtendkI = 0; // 0
    public static final double ExtendkD = 0.3; // 0.3
    public static final double ExtendkS = 2.4686; // 2.4686
    public static final double ExtendkV = 1; // 1
    public static final double ExtendkA = 1.01164; // 1.01164
    public static final double ExtendkG = 1; // 0.46724, 1

    public static final double RetractkP = 5.0;
    public static final double RetractkI = 0;
    public static final double RetractkD = 0.3;
    public static final double RetractkS = 2.4686;
    public static final double RetractkV = 1;
    public static final double RetractkA = 1.01164;
    public static final double RetractkG = 1.5;

    public static final double DownkP = 1.51337; // 1.51227
    public static final double DownkI = 0; // 0
    public static final double DownkD = 0.3; // 0.3
    public static final double DownkS = 2.4686; // 2.4686
    public static final double DownkV = 1; // 1
    public static final double DownkA = 1.01164; // 1.01164
    public static final double DownkG = 1; // 0.46724, 1

    // Simulation
    public static final double kSimArmGearRatio = 15.0;
    public static final double kSimArmMOI = 0.1; // kg*m^2 (mechanism side)
    public static final double kSimRollerGearRatio = 1.33; // also a planetary
    public static final double kSimRollerMOI = 0.001; // kg*m^2
  }

  // Shooter Motor Ids = 40s
  public static class ShooterConstants {
    // Facing from the front of the shooters (For Left and Right)
    // Front is the closest to the edge, Back is the furthest inward
    public static final int kLeftShooterId = 41;
    public static final int kRightShooterId = 42;

    // PID Constants for Shooting
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.12;
    public static final double kS = 0.1;

    /*
     * Desired Rotations per second for shooter Motors
     * WARNING: CURRENTLY TEMPORARY NUMBERS
     */
    public static final double shootRev = 85;
    public static final double passRev = 1;

    // Simulation (4:1 gearing, each motor drives 2 shafts)
    // Shaft A: 2x stealth(0.000141) + 1x SS flywheel(0.000790) = 0.00107
    // Shaft B: 2x stealth(0.000141) = 0.00028
    // Total mechanism-side MOI per motor = 0.00135 kg*m^2
    public static final double kSimGearRatio = 1.29;
    public static final double kSimMOI = 0.002; // kg*m^2
  }

  public static final class Vision {
    // Limelight camera names — must match the NetworkTables table name for each camera.
    // Camera-to-robot transforms are configured on the Limelight hardware, not in code.
    public static final String kRightCameraName = "limelight-right";
    public static final String kForwardCameraName = "limelight-forward";
    public static final String kReverseCameraName = "limelight-reverse";

    // Dynamic std-dev scaling constants
    // Formula: stdDev = base * (avgDistance ^ exponent)
    // If the robot snaps/jumps to vision poses too aggressively → increase the base
    // If vision corrections feel sluggish or ignored → decrease the base
    public static final double kSingleTagBaseXYStdDev = 1.5; // base error in meters at 1m distance
    // Large but finite — Kalman gain rounds to ~0 without overflowing when multiplied
    // by distance factors. Effectively ignores single-tag heading.
    public static final double kSingleTagBaseHeadingStdDev = 10.0; // radians (~573°)
    public static final double kMultiTagBaseXYStdDev = 1.0; // base error in meters at 1m distance
    public static final double kMultiTagBaseHeadingStdDev =
        0.45; // base error in radians at 1m (~34 deg)
    // Use different exponents for single vs multi case because in multi tag you have more corners
    // of april tags (4 each) to rely on
    public static final double kSingleTagDistanceExponent = 2.0; // quadratic scaling
    public static final double kMultiTagDistanceExponent = 1.0; // linear scaling

    // Acceptance rules
    public static final int kMinAprilTagsForPose = 1;
    public static final double kMaxAcceptableSingleTagAmbiguity = 0.2;

    // Pose sanity / QC filters
    public static final double kFieldBoundaryMarginMeters =
        0.5; // allow slightly outside field edge
    public static final double kMaxSingleTagDistanceMeters =
        2.5; // max reliable single-tag range (MT1)
    public static final double kMaxSingleTagDistanceMT2Meters =
        4.0; // MegaTag2 is reliable at greater range

    // Speed filters — reject vision when robot is moving too fast (motion blur)
    // These will only filter out extreme motion blur. Can tighten based on testing.
    public static final double kMaxVisionTranslationSpeed = 4.0; // m/s
    public static final double kMaxVisionRotationSpeed =
        Math.toRadians(720); // rad/s (2 full rotations)

    // Cross-sensor sanity gates applied in CommandSwerveDrivetrain.periodic().
    // Bypassed while disabled so pose can warm up / snap in on the field.
    // Reject vision if it disagrees with odometry XY by more than this:
    public static final double kMaxPoseJumpMeters = 1.0;
    // Reject vision if heading disagrees with gyro by more than this:
    public static final double kMaxHeadingDisagreementRad = Math.toRadians(30);

    /** Pipeline index 0 is the AprilTag localization pipeline on all three cameras. */
    public static final int kAprilTagPipeline = 0;

    /**
     * Seconds without a heartbeat increment before a camera is flagged as disconnected. At 20 ms /
     * cycle this is 50 cycles — long enough to survive a busy loop but short enough to catch a
     * genuinely dead camera before the match ends.
     */
    public static final double kHeartbeatTimeoutSec = 1.0;

    // Default enable states — flip to false here to disable vision globally or per-camera
    // at startup without touching subsystem code. Useful during development and debugging.
    public static final boolean kDefaultAllCamerasEnabled = true;
    public static final boolean kDefaultCameraEnabled = true;
  }

  public static class AimBotConstants {
    // Red alliance hub field coordinates (inches, converted to meters)
    public static final double kRedHubXMeters = Units.inchesToMeters(469.11);
    public static final double kRedHubYMeters = Units.inchesToMeters(158.845);

    // Heading PID constants for aiming at the hub
    public static final double kP = 7.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
