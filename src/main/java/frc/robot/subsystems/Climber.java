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
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX m_ClimberLeftMotor;

  private TalonFX m_ClimberRightMotor;

  public ClimberState m_ClimberState;

  private TalonFXConfiguration m_TalonFXConfig;

  public final double GearRatio = 0.16; // place holder
  private final double kSproketCircumference = 2 * Math.PI; // place holder

  public enum ClimberState {
    S_Hold,
    S_Extend,
    S_Lock,
    S_Retract
  }

  public Climber() {

    m_ClimberLeftMotor = new TalonFX(Constants.ClimberConstants.kClimberLeftMotor);
    m_ClimberRightMotor = new TalonFX(Constants.ClimberConstants.kClimberRightMotor);

    m_ClimberState = ClimberState.S_Hold;

    var fx_cfg = new MotorOutputConfigs();

    fx_cfg.NeutralMode = NeutralModeValue.Brake;

    m_ClimberLeftMotor.getConfigurator().apply(fx_cfg);
    m_ClimberRightMotor.getConfigurator().apply(fx_cfg);

    m_TalonFXConfig =
        new TalonFXConfiguration()
            .withVoltage(new VoltageConfigs().withPeakForwardVoltage(2).withPeakReverseVoltage(-2));

    m_ClimberLeftMotor.getConfigurator().apply(m_TalonFXConfig);
    m_ClimberRightMotor.getConfigurator().apply(m_TalonFXConfig);

    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kS = 0;
    Slot0Configs.kG = 0;
    Slot0Configs.kP = 0.1;
    Slot0Configs.kI = 0.1;
    Slot0Configs.kD = 0;

    m_ClimberLeftMotor.getConfigurator().apply(Slot0Configs);

    m_ClimberRightMotor.setControl(
        new Follower(m_ClimberLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    m_ClimberLeftMotor.setPosition(0);
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
    m_ClimberLeftMotor.setVoltage(0);
  }

  // Sets to the extended position, checks if set point reached then will switch to extend, and
  // holds position
  public void Extend() {
  
    setPosition(inchesToRotation(ClimberConstants.kExtensionPosition));
    if (SetpointReached(ClimberConstants.kExtensionPosition)) m_ClimberState = ClimberState.S_Hold;
  }

  // Sets to the lock position, checks if set point reached then will switch to lock, and holds
  // position
  public void Lock() {
    setPosition(inchesToRotation(ClimberConstants.kLockPosition));
    if (SetpointReached(ClimberConstants.kLockPosition)) m_ClimberState = ClimberState.S_Hold;
  }

  // Sets to the retract position, checks if set point reached then will switch to retract, and
  // holds position
  public void Retract() {

    setPosition(inchesToRotation(ClimberConstants.kRetractPosition));
    if (SetpointReached(ClimberConstants.kRetractPosition)) m_ClimberState = ClimberState.S_Hold;
  }

  public double rotationsToInches(double angle) {
    return angle * kSproketCircumference;
  }

  // Position = Current distance from lock (inches)
  public double LimitCheck() {
    return rotationsToInches(m_ClimberLeftMotor.getPosition().getValueAsDouble() / GearRatio);
  }

  public void setPosition(double position) {
    PositionVoltage pos = new PositionVoltage(position).withSlot(0);

    m_ClimberLeftMotor.setControl(pos);
  }

  public double inchesToRotation(double Length) {
    return (Length * GearRatio) / kSproketCircumference;
  }

  // Setpoint inches extended from lock (desired location)
  // WARNING: 1s are placeholders (Act as our error from location)
  public boolean SetpointReached(double Setpoint) {
    return (LimitCheck() - 1 <= Setpoint) && (LimitCheck() + 1 >= Setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Shuffleboard.getTab("Test");
    SmartDashboard.putNumber("ClimberMotorPosition", LimitCheck());
    SmartDashboard.putNumber("ClimberMotorRaw", m_ClimberLeftMotor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean(
        "SetpointReached", SetpointReached(Constants.ClimberConstants.kExtensionPosition));
    SmartDashboard.putString("ClimberState", m_ClimberState.toString());
    runClimberState();
  }
}
