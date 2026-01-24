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
    public static final int kCoDriverControllerPort = 1;
  }

  // Agitator IDs : 20s
  public static class AgitatorConstants {
    public static final int kAgitatorMotor = 21;
    // 4 is a placeholder value for motor voltage
    public static final double kAgitatorVolt = 4;
  }
  public static class FeederConstants {
    public static final int kFeederMotor = 31;
  }

  public static class ClimberConstants {
    public static final int kClimberLeftMotor = 62;
    public static final int kClimberRightMotor = 61;
    public static final double kExtensionPosition = 300; // placeholder in inches
    public static final double kRetractPosition = 3; // placeholder in inches
    public static final double kLockPosition = 0;
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

    /*Desired Rotations per second for shooter Motors
    WARNING: CURRENTLY TEMPORARY NUMBERS*/
    public static final double shootRev = 10;
    public static final double passRev = 5;
  }
}
