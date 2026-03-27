// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static class OperatorConstants {
    public static final int kXboxControllerPort = 1;
    public static final int kPs4ControllerPort = 0;
  }

  // Agitator IDs : 20s
  public static class AgitatorConstants {
    public static final int kAgitatorLeftMotor = 21;
    public static final int kAgitatorRightMotor = 22;
    public static final double kAgitatorVoltIn = 4;
    public static final double kAgitatorVoltOut = -4;
    public static final double kAgitatorVoltIdle = 3;
  }

  // Feeder Motor Ids = 30s
  public static class FeederConstants {
    public static final int kFeederLeftMotor = 31;
    public static final int kFeederRightMotor = 32;
    // 3 is a placeholder for motor voltage
    public static final double kFeederVolt = 3;
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
    public static final int kIntakeMotorRoller1 = 12;
    public static final int kIntakeMotorRoller2 = 13;
    // The number of rotations using the falcon's encoder
    public static final double kExtensionPosition = 4.9; // 4.7
    public static final double kExtentionCycleUpPos = kExtensionPosition - 1.5;
    public static final double kRetractPosition = 0; // Intake is retract ed and in the bot

    // PID Constants for Intake Extension
    public static final double ExtendkP = 1.51337; // 1.51227
    public static final double ExtendkI = 0; // 0
    public static final double ExtendkD = 0.3; // 0.3
    public static final double ExtendkS = 2.4686; // 2.4686
    public static final double ExtendkV = 1; // 1
    public static final double ExtendkA = 1.01164; // 1.01164
    public static final double ExtendkG = 1; // 0.46724, 1

    public static final double RetractkP = 3.0;
    public static final double RetractkI = 0;
    public static final double RetractkD = 0.3;
    public static final double RetractkS = 2.4686;
    public static final double RetractkV = 1;
    public static final double RetractkA = 1.01164;
    public static final double RetractkG = 1;

    public static final double DownkP = 1.51337; // 1.51227
    public static final double DownkI = 0; // 0
    public static final double DownkD = 0.3; // 0.3
    public static final double DownkS = 2.4686; // 2.4686
    public static final double DownkV = 1; // 1
    public static final double DownkA = 1.01164; // 1.01164
    public static final double DownkG = 1; // 0.46724, 1
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
    public static final double shootRev = 75;
    public static final double passRev = 1;
  }

  public static final class Vision {
    // Right camera name
    // Must match camera set in PhotonVision UI at photonvision.local:5800
    public static final String kRightCameraName = "rightPhotonvisionCamera";
    // Robot -> Camera transform (camera pose relative to robot origin).
    // WPILib coordinate convention: +X forward, +Y left, +Z up.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // In most cases in WPILib programming, 0° is aligned with the positive X axis,
    // and 180° is aligned with the negative X axis. CCW rotation is positive, so
    // 90° is aligned with the positive Y axis, and -90° is aligned with the
    // negative Y axis.

    // robot coordination system is centered on the floor. So a camer at the center, facing forward
    // on the floor, is 0,0,0,0,0,0

    //  two on mast:

    // one backwards (0,-2.78,24.75,0,0,0)
    // or
    // one backwards (0,-2.78,24.75)
    // one facing right rotated  90 CW  (top pointing intake) (must match photonvision ui) in )

    public static final double kRightCameraMountPitchAngleRad = Units.degreesToRadians(0);
    public static final double kRightCameraMountRollAngleRad = Units.degreesToRadians(-90);
    public static final double kRightCameraMountYawAngleRad = Units.degreesToRadians(-90);

    public static final double kRightCameraForwardMeters = Units.inchesToMeters(-3.78);
    public static final double kRightCameraLeftMeters = Units.inchesToMeters(-3);
    public static final double kRightCameraUpMeters = Units.inchesToMeters(22);

    // Robot to right camera transform
    public static final Transform3d kRightRobotToCam3d =
        new Transform3d(
            new Translation3d(
                kRightCameraForwardMeters, kRightCameraLeftMeters, kRightCameraUpMeters),
            new Rotation3d(
                kRightCameraMountRollAngleRad,
                kRightCameraMountPitchAngleRad,
                kRightCameraMountYawAngleRad));

    // Forward camera name
    // Must match camera set in PhotonVision UI
    public static final String kForwardCameraName = "forwardPhotonvisionCamera";

    public static final double kForwardCameraMountPitchAngleRad = Units.degreesToRadians(0);
    public static final double kForwardCameraMountRollAngleRad = Units.degreesToRadians(0);
    public static final double kForwardCameraMountYawAngleRad = Units.degreesToRadians(0);

    public static final double kForwardCameraForwardMeters = Units.inchesToMeters(0);
    public static final double kForwardCameraLeftMeters = Units.inchesToMeters(0);
    public static final double kForwardCameraUpMeters = Units.inchesToMeters(22);

    // Robot to forward camera transform
    public static final Transform3d kForwardRobotToCam3d =
        new Transform3d(
            new Translation3d(
                kForwardCameraForwardMeters, kForwardCameraLeftMeters, kForwardCameraUpMeters),
            new Rotation3d(
                kForwardCameraMountRollAngleRad,
                kForwardCameraMountPitchAngleRad,
                kForwardCameraMountYawAngleRad));

    // Dynamic std-dev scaling constants
    // Formula: stdDev = base * (avgDistance ^ exponent)
    // If the robot snaps/jumps to vision poses too aggressively → increase the base
    // If vision corrections feel sluggish or ignored → decrease the base
    public static final double kSingleTagBaseXYStdDev = 0.5; // base error in meters at 1m distance
    public static final double kSingleTagBaseHeadingStdDev =
        Double.MAX_VALUE; // don't trust single-tag heading
    public static final double kMultiTagBaseXYStdDev = 0.3; // base error in meters at 1m distance
    public static final double kMultiTagBaseHeadingStdDev =
        0.1; // base error in radians at 1m (~5.7 deg)
    // Use different exponents for single vs multi case because in multi tag you have more corners
    // of april tags (4 each) to rely on
    public static final double kSingleTagDistanceExponent = 2.0; // quadratic scaling
    public static final double kMultiTagDistanceExponent = 1.0; // linear scaling

    // Acceptance rules
    public static final int kMinAprilTagsForPose = 1;
    public static final double kMaxAcceptableSingleTagAmbiguity = 0.2;

    // Pose sanity / QC filters
    public static final double kMaxPoseHeightMeters =
        0.75; // reject if estimated Z is off the floor
    public static final double kFieldBoundaryMarginMeters =
        0.5; // allow slightly outside field edge
    public static final double kMaxSingleTagDistanceMeters = 4.0; // max reliable single-tag range

    // Speed filters — reject vision when robot is moving too fast (motion blur)
    // These will only filter out extreme motion blur. Can tighten based on testing.
    public static final double kMaxVisionTranslationSpeed = 4.0; // m/s
    public static final double kMaxVisionRotationSpeed =
        Math.toRadians(720); // rad/s (2 full rotations)

    // Simulation camera properties (Limelight 4 — OV9782, HFOV 82°, VFOV ~56°)
    // Two resolutions are available (640x400 and 1280x800)
    public static final int kSimCameraResWidth = 1280;
    public static final int kSimCameraResHeight = 800;
    public static final double kSimCameraFOVDeg = 91.4; // diagonal FOV derived from 82° HFOV
    public static final double kSimAvgLatencyMs = 35.0;
    public static final double kSimLatencyStdDevMs = 5.0;
    public static final int kSimCameraFPS = 20;
  }

  public static class AimBotConstants {
    // Red alliance hub field coordinates (inches, converted to meters)
    public static final double kRedHubXMeters = Units.inchesToMeters(158.61);
    public static final double kRedHubYMeters = Units.inchesToMeters(158.845);

    // Heading PID constants for aiming at the hub
    public static final double kP = 7.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
