// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_IntakeMotorArm;

  private TalonFX m_IntakeMotorRoller;

  public IntakeState intakeState;

  private double FORWARD_LIMIT = 100; // Placeholder
  private double REVERSE_LIMIT = 0;

  public Intake() {
    // Created Two Motors
    m_IntakeMotorArm = new TalonFX(IntakeConstants.kIntakeMotorArm);
    m_IntakeMotorRoller = new TalonFX(IntakeConstants.kIntakeMotorRoller);

    // Configs that use the PID values to help with motor speed
    var talonFXConfigs =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(FORWARD_LIMIT)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(REVERSE_LIMIT)
                    .withReverseSoftLimitEnable(true));

    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = Constants.IntakeConstants.kP;
    Slot0Configs.kI = Constants.IntakeConstants.kI;
    Slot0Configs.kD = Constants.IntakeConstants.kD;
    Slot0Configs.kV = Constants.IntakeConstants.kV;
    Slot0Configs.kA = Constants.IntakeConstants.kA;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        Constants.IntakeConstants.MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.IntakeConstants.MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = Constants.IntakeConstants.MotionMagicJerk;

    // Apply the Configs to the Motor Objects
    m_IntakeMotorArm.getConfigurator().apply(talonFXConfigs);
    m_IntakeMotorArm.getConfigurator().apply(Slot0Configs);

    m_IntakeMotorArm.setPosition(0);
  }

  public enum IntakeState {
    S_Extend,
    S_Retract,
    S_Outtake,
    S_Extended,
    S_Retracted
  }

  public void RunIntakeState() {
    switch (intakeState) {
      case S_Extend:
        ExtendPosition();
        break;
      case S_Retract:
        RetractPosition();
        break;
      case S_Outtake:
        Outtake();
        break;
    }
  }

  // returns the Arm Position as a double (rotations)
  public double getPosition() {
    return m_IntakeMotorArm.getPosition().getValueAsDouble() / IntakeConstants.GearRatio;
  }

  // Gives the motor velocity using arm position
  public void setPosition(double position) {
    PositionVoltage pos = new PositionVoltage(position * IntakeConstants.GearRatio).withSlot(0);
    m_IntakeMotorArm.setControl(pos);
  }

  // A method that returns true if the arm is at its destination
  public boolean checkExtended(double Setpoint) {
    double position = getPosition();
    return (position - 1 / 180 <= Setpoint)
        && (position + 1 / 180 >= Setpoint); // subject to change
  }

  // Extends the intake out and starts the rollers
  public void ExtendPosition() {
    // If motor has reached its destination the stop the arm and start the rollers
    if (checkExtended(IntakeConstants.kExtensionPosition)) {
      m_IntakeMotorArm.setVoltage(0);
      m_IntakeMotorRoller.setVoltage(4);
      intakeState = intakeState.S_Extended;
    } else {
      // Move the arm until it reaches its destination
      setPosition(IntakeConstants.kExtensionPosition);
    }
  }

  // Pulls the intake back in and stops the rollers
  public void RetractPosition() {
    // When retracting we want to rollers to stay off
    m_IntakeMotorRoller.setVoltage(0);
    // If the arm has reached its destination stop the motor
    if (checkExtended(IntakeConstants.kRetractPosition)) {
      m_IntakeMotorArm.setVoltage(0);
      intakeState = intakeState.S_Retracted;
    } else {
      // Move the arm until it reaches the destination
      setPosition(IntakeConstants.kRetractPosition);
    }
  }

  // Extends the intake if it is not out and reverses the rollers to eject pieces
  public void Outtake() {
    // If motor has reached its destination the stop the arm and start the rollers moving backwards
    // (ejecting)
    if (checkExtended(IntakeConstants.kExtensionPosition)) {
      m_IntakeMotorArm.setVoltage(0);
      m_IntakeMotorRoller.setVoltage(-4);
    } else {
      // Move the arm until it reaches its destination
      setPosition(IntakeConstants.kExtensionPosition);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CheckExtended", checkExtended(IntakeConstants.kExtensionPosition));
    SmartDashboard.putBoolean("CheckRetracted", checkExtended(IntakeConstants.kRetractPosition));
    SmartDashboard.putNumber("ForwardSoftLimit", FORWARD_LIMIT);
    SmartDashboard.putNumber("ReverseSoftLimit", REVERSE_LIMIT);
    SmartDashboard.putNumber(
        "RawIntakePosition", m_IntakeMotorArm.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakePose", getPosition());
    // This method will be called once per scheduler run
    RunIntakeState();
  }
}
