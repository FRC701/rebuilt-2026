// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Agitator extends SubsystemBase {
  private TalonFX m_AgitatorLeftMotor;
  private TalonFX m_AgitatorRightMotor;

  public AgitatorState m_AgitatorState;

  /** Creates a new Aggitator. */
  public Agitator() {
    m_AgitatorLeftMotor = new TalonFX(Constants.AgitatorConstants.kAgitatorLeftMotor);
    m_AgitatorRightMotor = new TalonFX(Constants.AgitatorConstants.kAgitatorRightMotor);

    m_AgitatorState = AgitatorState.S_On;

    m_AgitatorRightMotor.setControl(
        new Follower(m_AgitatorLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /*
   * S_On = motor is spinning at a constant speed
   * S_Off = motor is inactive
   */
  public enum AgitatorState {
    S_On,
    S_Off
  }

  public AgitatorState mAgitatorState;

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
    m_AgitatorLeftMotor.setVoltage(Constants.AgitatorConstants.kAgitatorVolt);
  }

  // method for when motor is not in motion
  public void stopAgitatorMotor() {
    m_AgitatorLeftMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    runAgitatorState();
  }
}
