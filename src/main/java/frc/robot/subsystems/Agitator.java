// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    m_AgitatorState = AgitatorState.S_Off;

    m_AgitatorRightMotor.setControl(
        new Follower(m_AgitatorLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    var m_TalonFXConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40));

    m_AgitatorLeftMotor.getConfigurator().apply(m_TalonFXConfig);
  }

  /*
   * S_On = motor is spinning at a constant speed
   * S_Off = motor is inactive
   */
  public enum AgitatorState {
    S_In,
    S_Off,
    S_Out,
    S_Idle
  }

  public void runAgitatorState() {
    switch (m_AgitatorState) {
      case S_In:
        AgitatorIntaking();
        break;
      case S_Off:
        AgitatorStop();
        break;
      case S_Out:
        AgitatorOuttaking();
        break;
      case S_Idle:
        AgitatorIdling();
        break;
    }
  }

  // method for when motor is in motion
  public void AgitatorIntaking() {
    m_AgitatorLeftMotor.setVoltage(Constants.AgitatorConstants.kAgitatorVoltIn);
  }

  // method for when motor is not in motion
  public void AgitatorStop() {
    m_AgitatorLeftMotor.setVoltage(0);
  }

  public void AgitatorOuttaking() {
    m_AgitatorLeftMotor.setVoltage(Constants.AgitatorConstants.kAgitatorVoltIn);
  }

  public void AgitatorIdling() {
    m_AgitatorLeftMotor.setVoltage(Constants.AgitatorConstants.kAgitatorVoltIdle);
  }

  @Override
  public void periodic() {
    runAgitatorState();
  }
}
