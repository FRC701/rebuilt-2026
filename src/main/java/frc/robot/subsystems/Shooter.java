// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CandleSubsystem;

public class Shooter extends SubsystemBase {

  private TalonFX m_FrontMotor;
  private TalonFX m_BackMotor;

  public ShooterEnumState m_ShooterEnumState;

  public CandleSubsystem m_CandleSub;

  /** Creates a new Shooter. */
  public Shooter(int frontId, int backId, CandleSubsystem candleSub) {
    // Selects the intial state
    m_ShooterEnumState = ShooterEnumState.S_Shooting;

    var m_TalonFXConfig =
        new TalonFXConfiguration()
            .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6).withPeakReverseVoltage(-6));

    // Configs that use the PID values to help with motor speed
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = Constants.ShooterConstants.kP;
    Slot0Configs.kI = Constants.ShooterConstants.kI;
    Slot0Configs.kD = Constants.ShooterConstants.kD;
    Slot0Configs.kV = Constants.ShooterConstants.kV;
    Slot0Configs.kA = Constants.ShooterConstants.kA;

    // Identifying of the motors and making the front one the leader
    m_FrontMotor = new TalonFX(frontId);
    m_BackMotor = new TalonFX(backId);
    m_BackMotor.setControl(new Follower(m_FrontMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Applying the configs to the motors, PID
    m_FrontMotor.getConfigurator().apply(Slot0Configs);
    m_BackMotor.getConfigurator().apply(Slot0Configs);

    // Applying the configs to the motors, Voltage Limits
    m_FrontMotor.getConfigurator().apply(m_TalonFXConfig);
    m_BackMotor.getConfigurator().apply(m_TalonFXConfig);

    m_CandleSub = candleSub;
  }

  // Uses PID to arrive at our shooting speed
  public void shooting() {
    // voltSpeed = desired amount of rotations per second
    VelocityVoltage voltSpeed =
        new VelocityVoltage(Constants.ShooterConstants.shootRev).withSlot(0);
    m_FrontMotor.setControl(voltSpeed);
  }

  // Uses PID to arrive at our passing speed
  public void passing() {
    // voltSpeed = desired amount of rotations per second
    VelocityVoltage voltSpeed = new VelocityVoltage(Constants.ShooterConstants.passRev).withSlot(0);
    m_FrontMotor.setControl(voltSpeed);
  }

  // Sets the speed to 0 by creating a VelocityVotage object with 0 velocity
  public void stopping() {
    m_FrontMotor.setControl(new VelocityVoltage(0).withSlot(0));
  }

  public enum ShooterEnumState {
    S_Shooting,
    S_Passing,
    S_NotShooting
  }

  public void runShooterStates() {
    switch (m_ShooterEnumState) {
      case S_Shooting:
        shooting();
        m_CandleSub.shooterLED();
        break;
      case S_Passing: 
        passing();
        m_CandleSub.passingLED();
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
