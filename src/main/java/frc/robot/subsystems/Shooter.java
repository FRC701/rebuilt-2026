// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public TalonFX frontMotor;

  public static ShooterEnumState mShooterEnumState;

  /** Creates a new Shooter. */
  public Shooter(int frontId) {
    mShooterEnumState = ShooterEnumState.S_Shooting;
    frontMotor = new TalonFX(frontId);
  }

  public enum ShooterEnumState {
    S_Shooting,
    S_Passing,
    S_NotShooting
  }

  public void runShooterStates() {
    switch (mShooterEnumState) {
      case S_Shooting:

      break;
      case S_Passing:

      break;
      case S_NotShooting:

      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runShooterStates();
  }
}
