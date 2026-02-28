// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX m_ClimberLeader;

  private TalonFX m_ClimberFollower;

  public ClimberState m_ClimberState;

  private TalonFXConfiguration m_TalonFXConfig;

  public final double GearRatio = 6.1;
  private final double kSproketCircumference = 2 * Math.PI;

  public enum ClimberState {
    S_Hold,
    S_Extend,
    S_Lock,
    S_Retract
  }

  public Climber() {

    m_ClimberLeader = new TalonFX(Constants.ClimberConstants.kClimberLeader);
    m_ClimberFollower = new TalonFX(Constants.ClimberConstants.kClimberFollower);

    m_ClimberState = ClimberState.S_Hold;

    var fx_cfg = new MotorOutputConfigs();

    fx_cfg.NeutralMode = NeutralModeValue.Brake;

    m_ClimberLeader.getConfigurator().apply(fx_cfg);
    m_ClimberFollower.getConfigurator().apply(fx_cfg);

    m_TalonFXConfig =
        new TalonFXConfiguration()
            .withVoltage(
                new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12));

    m_ClimberLeader.getConfigurator().apply(m_TalonFXConfig);
    m_ClimberFollower.getConfigurator().apply(m_TalonFXConfig);

    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kS = Constants.ClimberConstants.kS;
    Slot0Configs.kG = Constants.ClimberConstants.kG;
    Slot0Configs.kP = Constants.ClimberConstants.kP;
    Slot0Configs.kI = Constants.ClimberConstants.kI;
    Slot0Configs.kD = Constants.ClimberConstants.kD;

    m_ClimberLeader.getConfigurator().apply(Slot0Configs);

    m_ClimberFollower.setControl(
        new Follower(m_ClimberLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    m_ClimberLeader.setPosition(0);
  }

  public void runClimberState() {
    switch (m_ClimberState) {
      case S_Hold:
        HoldPosition();
        break;
      case S_Extend:
        Extend();
        break;
      case S_Lock:
        Lock();
        break;
      case S_Retract:
        Retract();
        break;
    }
  }

  public void HoldPosition() {
    m_ClimberLeader.setVoltage(0);
  }

  // Sets to the extended position, checks if set point reached then will switch to extend, and
  // holds position
  public void Extend() {

    setPosition(inchesToRotation(Constants.ClimberConstants.kExtensionPosition));
    if (SetpointReached(Constants.ClimberConstants.kExtensionPosition))
      m_ClimberState = ClimberState.S_Hold;
  }

  // Sets to the lock position, checks if set point reached then will switch to lock, and holds
  // position
  public void Lock() {
    setPosition(inchesToRotation(Constants.ClimberConstants.kLockPosition));
    if (SetpointReached(Constants.ClimberConstants.kLockPosition))
      m_ClimberState = ClimberState.S_Hold;
  }

  // Sets to the retract position, checks if set point reached then will switch to retract, and
  // holds position
  public void Retract() {
    setPosition(inchesToRotation(Constants.ClimberConstants.kRetractPosition));
  }

  public double rotationsToInches(double angle) {
    return angle * kSproketCircumference;
  }

  // Position = Current distance from lock (inches)
  public double LimitCheck() {
    return rotationsToInches(m_ClimberLeader.getPosition().getValueAsDouble() / GearRatio);
  }

  public void setPosition(double position) {
    PositionVoltage pos = new PositionVoltage(position).withSlot(0);

    m_ClimberLeader.setControl(pos);
  }

  public double inchesToRotation(double Length) {
    return (Length * GearRatio) / kSproketCircumference;
  }

  // Setpoint inches extended from lock (desired location)
  // WARNING: 1s are placeholders (Act as our error from location)
  public boolean SetpointReached(double Setpoint) {
    return (LimitCheck() - 0.1 <= Setpoint) && (LimitCheck() + 0.1 >= Setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Shuffleboard.getTab("Test");
    SmartDashboard.putNumber("ClimberMotorPosition", LimitCheck());
    SmartDashboard.putNumber("ClimberMotorRaw", m_ClimberLeader.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean(
        "SetpointReached", SetpointReached(Constants.ClimberConstants.kExtensionPosition));
    SmartDashboard.putString("ClimberState", m_ClimberState.toString());
    runClimberState();
  }
}
