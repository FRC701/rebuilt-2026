// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;

public class Shooter extends SubsystemBase {

  private TalonFX frontMotor;
  private TalonFX backMotor;

  /** Creates a new Shooter. */
  public Shooter(int frontId, int backId) {
    frontMotor = new TalonFX(frontId);
    backMotor = new TalonFX(backId);
    frontMotor.setControl(new Follower(backId,true));
  }

  

  private void setSpeed(double voltage){
    frontMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
