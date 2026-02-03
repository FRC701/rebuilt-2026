// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Agitator extends SubsystemBase {

  private TalonFX m_AgitatorMotor;

  public static AgitatorState m_AgitatorState;

  /** Creates a new Aggitator. */
  public Agitator() {
    m_AgitatorMotor = new TalonFX(Constants.AgitatorConstants.kAgitatorMotor);
    m_AgitatorState = AgitatorState.S_On;
  }

  /*
   * S_On = motor is spinning at a constant speed
   * S_Off = motor is inactive
   */
  public enum AgitatorState {
    S_On,
    S_Off
  }

  public void runAgitatorState() {
    switch (m_AgitatorState) {
      case S_On:
        spinAgitatorMotor();
        break;
      case S_Off:
        stopAgitatorMotor();
        break;
    }
  }

  // method for when motor is in motion
  public void spinAgitatorMotor() {
    m_AgitatorMotor.setVoltage(Constants.AgitatorConstants.kAgitatorVolt);
  }

  // method for when motor is not in motion
  public void stopAgitatorMotor() {
    m_AgitatorMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    runAgitatorState();
  }
}
