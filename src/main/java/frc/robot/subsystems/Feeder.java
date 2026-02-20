// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public FeederState m_FeederState;

  private TalonFX m_FeederMotor;

  public Feeder(int motorID) {
    m_FeederMotor = new TalonFX(motorID);
    m_FeederState = FeederState.S_Off;
  }

  public void runFeederState() {
    switch (m_FeederState) {
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
    m_FeederMotor.setVoltage(Constants.FeederConstants.kFeederVolt);
  }

  // method for when motor is not in motion
  public void stopFeederMotor() {
    m_FeederMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runFeederState();
  }
}
