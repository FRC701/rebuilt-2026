// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public FeederState m_FeederState;

  private TalonFX m_FeederMotor;

  public Feeder(int motorID, String feederName) {
    m_FeederMotor = new TalonFX(motorID);

        var m_TalonFXConfig =
        new TalonFXConfiguration();

        m_FeederState = FeederState.S_Off;

     MotorOutputConfigs feederConfig = m_TalonFXConfig.MotorOutput;

if (motorID == Constants.FeederConstants.kFeederLeftMotor) {
      feederConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      feederConfig.Inverted = InvertedValue.Clockwise_Positive;
    }

    m_FeederMotor.getConfigurator().apply(m_TalonFXConfig);
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
