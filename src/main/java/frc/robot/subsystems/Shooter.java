// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX mFrontMotor;
  private TalonFX mBackMotor;

  public ShooterEnumState mShooterEnumState;

  /** Creates a new Shooter. */
  public Shooter(int frontId, int backId) {
    mShooterEnumState = ShooterEnumState.S_Shooting;

    // Configs that use the PID values to help with motor speed
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = Constants.ShooterConstants.kP;
    Slot0Configs.kI = Constants.ShooterConstants.kI;
    Slot0Configs.kD = Constants.ShooterConstants.kD;
    Slot0Configs.kV = Constants.ShooterConstants.kV;
    Slot0Configs.kA = Constants.ShooterConstants.kA;

    // Identifying of the motors and making the front one the leader
    mFrontMotor = new TalonFX(frontId);
    mBackMotor = new TalonFX(backId);
    mBackMotor.setControl(new Follower(mFrontMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Applying the configs to the motors
    mFrontMotor.getConfigurator().apply(Slot0Configs);
    mBackMotor.getConfigurator().apply(Slot0Configs);
  }

  // Uses PID to arrive at our shooting speed
  public void shooting() {
    // voltSpeed = desired amount of rotations per second
    VelocityVoltage voltSpeed =
        new VelocityVoltage(Constants.ShooterConstants.shootRev).withSlot(0);
    mFrontMotor.setControl(voltSpeed);
  }

  // Uses PID to arrive at our passing speed
  public void passing() {
    // voltSpeed = desired amount of rotations per second
    VelocityVoltage voltSpeed = new VelocityVoltage(Constants.ShooterConstants.passRev).withSlot(0);
    mFrontMotor.setControl(voltSpeed);
  }

  // Sets the speed to 0 by creating a VelocityVotage object with 0 velocity
  public void stopping() {
    mFrontMotor.setControl(new VelocityVoltage(0).withSlot(0));
  }

  public enum ShooterEnumState {
    S_Shooting,
    S_Passing,
    S_NotShooting
  }

  public void runShooterStates() {
    switch (mShooterEnumState) {
      case S_Shooting:
        shooting();
        break;
      case S_Passing:
        passing();
        break;
      case S_NotShooting:
        stopping();
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runShooterStates();
  }
}
