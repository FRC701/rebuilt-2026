// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorArm = 11;
    public static final int kIntakeMotorRoller = 12;
    // public static final int kIntakeMotorArm2 = 13;
    // public static final int kIntakeMotorRoller2 = 14;
    // The number of rotations until fully extended from retract position (in degrees)
    public static final double kExtensionPosition = 37.5 / 360;
    public static final int kRetractPosition = 0; // Intake is retracted and in the bot
    public static final double GearRatio = 88 / 3; // Arm Teeth:Motor Teeth

    // PID Constants for Intake Extension
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.1;
    public static final double kA = 100;
  }

  // Shooter Motor Ids = 40s
  public static class ShooterConstants {
    // Facing from the front of the shooters (For Left and Right)
    // Front is the closest to the edge, Back is the furthest inward
    public static final int kFrontLeftShooterId = 41;
    public static final int kBackLeftShooterId = 42;
    public static final int kFrontRightShooterId = 43;
    public static final int kBackRightShooterId = 44;

    // PID Constants for Shooting
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kV = 0.1;
    public static final double kA = 100;

    /*Desired Rotations per second for shooter Motors
    WARNING: CURRENTLY TEMPORARY NUMBERS*/
    public static final double shootRev = 2;
    public static final double passRev = 1;
  }
}
