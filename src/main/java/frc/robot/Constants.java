// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  // Agitator IDs : 20s
  public static class AgitatorConstants {
    public static final int kAgitatorLeftMotor = 21;
    public static final int kAgitatorRightMotor = 22;
    public static final double kAgitatorVolt = 4;
  }

  // Feeder Motor Ids = 30s
  public static class FeederConstants {
    public static final int kFeederMotor = 31;
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
    public static final double shootRev = 2;
    public static final double passRev = 1;
  }

  public static final class Vision {
    // Forward camera name
    // Must match camera set in PhotonVision UI at photonvision.local:5800
    public static final String kForwardCameraName = "forwardPhotonvisionCamera";
    // Robot -> Camera transform (camera pose relative to robot origin).
    // WPILib coordinate convention: +X forward, +Y left, +Z up.
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    // In most cases in WPILib programming, 0° is aligned with the positive X axis,
    // and 180° is aligned with the negative X axis. CCW rotation is positive, so
    // 90° is aligned with the positive Y axis, and -90° is aligned with the
    // negative Y axis.

    public static final double kForwardCameraMountPitchAngleRad = Units.degreesToRadians(0);
    public static final double kForwardCameraMountRollAngleRad = Units.degreesToRadians(0);
    public static final double kForwardCameraMountYawAngleRad = Units.degreesToRadians(0);

    public static final double kForwardCameraForwardMeters = Units.inchesToMeters(11);
    public static final double kForwardCameraLeftMeters = Units.inchesToMeters(11);
    public static final double kForwardCameraUpMeters = Units.inchesToMeters(5.504);

    // Robot to forward camera transform
    public static final Transform3d kForwardRobotToCam3d =
        new Transform3d(
            new Translation3d(
                kForwardCameraForwardMeters, kForwardCameraLeftMeters, kForwardCameraUpMeters),
            new Rotation3d(
                kForwardCameraMountRollAngleRad,
                kForwardCameraMountPitchAngleRad,
                kForwardCameraMountYawAngleRad));

    // Vision trust assumptions (1σ)
    public static final double kVisionPosStdDevMeters = 0.7;
    public static final double kVisionHeadingStdDevDeg = 10.0;

    public static final Matrix<N3, N1> kVisionStdDevs =
        VecBuilder.fill(
            kVisionPosStdDevMeters,
            kVisionPosStdDevMeters,
            Math.toRadians(kVisionHeadingStdDevDeg));

    // Acceptance rules
    public static final int kMinAprilTagsForPose = 1;
    public static final double kMaxAcceptableSingleTagAmbiguity = 0.25;

    // Pose sanity / QC filters
    public static final double kMaxPoseHeightMeters =
        0.75; // reject if estimated Z is off the floor
    public static final double kFieldBoundaryMarginMeters =
        0.5; // allow slightly outside field edge
    public static final double kMaxSingleTagDistanceMeters = 4.0; // max reliable single-tag range
  }
}
