// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX m_ShooterMotor;

  public ShooterEnumState m_ShooterEnumState;

  // voltSpeed = desired amount of rotations per second
  private VelocityVoltage voltSpeed = new VelocityVoltage(0).withSlot(0);

  private StatusSignal<AngularVelocity> velocitySignal;

  /** Creates a new Shooter. */
  public Shooter(int motorId) {
    // Selects the intial state
    m_ShooterEnumState = ShooterEnumState.S_NotShooting;

    var m_TalonFXConfig =
        new TalonFXConfiguration()
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10));

    MotorOutputConfigs shooterConfig = new MotorOutputConfigs();
    shooterConfig.Inverted = InvertedValue.Clockwise_Positive;

    // Configs that use the PID values to help with motor speed
    Slot0Configs Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = Constants.ShooterConstants.kP;
    Slot0Configs.kI = Constants.ShooterConstants.kI;
    Slot0Configs.kD = Constants.ShooterConstants.kD;
    Slot0Configs.kV = Constants.ShooterConstants.kV;
    Slot0Configs.kS = Constants.ShooterConstants.kS;
    m_TalonFXConfig.withSlot0(Slot0Configs);

    // Identifying of the motors and making the front one the leader
    m_ShooterMotor = new TalonFX(motorId);

    // Applying the configs to the motors, PID
    // Applying the configs to the motors, Voltage Limits
    m_ShooterMotor.getConfigurator().apply(m_TalonFXConfig);

    m_ShooterMotor.getConfigurator().apply(shooterConfig);

    velocitySignal = m_ShooterMotor.getVelocity();
  }

  // Uses PID to arrive at our shooting speed
  public void shooting() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(Constants.ShooterConstants.shootRev));
  }

  // Uses PID to arrive at our passing speed
  public void passing() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(Constants.ShooterConstants.passRev));
  }

  // Sets the speed to 0 by using a VelocityVotage object with 0 velocity
  public void stopping() {
    m_ShooterMotor.setControl(voltSpeed.withVelocity(0));
  }

  public double VoltageCheck() {
    return m_ShooterMotor.getVelocity().getValueAsDouble();
  }

  // returns true if no balls in shooter
  public boolean CurrentCHeck() {
    return m_ShooterMotor.getStatorCurrent().getValueAsDouble() <= 1; // placeholder
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
    // Shuffleboard.getTab("ShooterTab").add("ShooterState", m_ShooterEnumState.toString());
    SmartDashboard.putString("ShooterState", m_ShooterEnumState.toString());
    SmartDashboard.putNumber(
        "RevolutionsError", m_ShooterMotor.getClosedLoopError().refresh().getValueAsDouble());
    SmartDashboard.putNumber("ShooterSpeed", velocitySignal.getValueAsDouble());
    velocitySignal.refresh();

    runShooterStates();
  }
}
