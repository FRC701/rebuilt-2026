// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX mClimberLeftMotor;

  private TalonFX mClimberRightMotor;

  public static ClimberState mClimberState;

  public enum ClimberState {
    S_Hold,
    S_Extend,
    S_Lock,
    S_Retract
  }

  public Climber() {
    mClimberLeftMotor = new TalonFX(Constants.ClimberConstants.kClimberLeftMotor);
    mClimberRightMotor = new TalonFX(Constants.ClimberConstants.kClimberRightMotor);
    mClimberRightMotor.setControl(
        new Follower(mClimberLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void runClimberState() {
    switch (mClimberState) {
      case S_Hold:
        HoldPosition();
        break;
      case S_Extend:
        Extend();
        break;
      case S_Lock:
        Lock();
        break;
      case S_Retract:
        break;
    }
  }

  public void HoldPosition() {}

  public void Extend() {}

  public void Lock() {}

  public void Retract() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
