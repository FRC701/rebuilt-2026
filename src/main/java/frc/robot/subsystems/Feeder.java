// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private TalonFX FeederMotor;

  public static FeederState mFeederState;

  public Feeder(int motorID) {
    FeederMotor = new TalonFX(motorID);
    mFeederState = FeederState.S_Off;
  }

  public void runFeederState() {
    switch (mFeederState) {
      case S_On:
        spinFeederMotor();
        break;
      case S_Off:
        stopFeederMotor();
        break;
    }
  }
  
  public enum FeederState {
    S_On,
    S_Off
  }

  // method for when motor is in motion
  public void spinFeederMotor() {
    FeederMotor.setVoltage(3);
  }
  
  // method for when motor is not in motion
  public void stopFeederMotor() {
    FeederMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runFeederState();
  }
}
