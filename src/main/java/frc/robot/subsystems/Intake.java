// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_IntakeMotorArm;

  private TalonFX m_IntakeMotorRoller;

  public IntakeState m_IntakeState;

  private double FORWARD_LIMIT = 4.7; // Placeholder
  private double REVERSE_LIMIT = 0;

  private final SysIdRoutine m_SysID =
      new SysIdRoutine(
          // Config default is 1 volt/sec, 7V step, 10 sec timeout
          new Config(Units.Volts.per(Units.Second).of(1), Units.Volts.of(7), Units.Seconds.of(10)),
          new Mechanism(this::voltageCallback, this::logCallback, this));

  public Intake() {
    // Created Two Motors
    m_IntakeMotorArm = new TalonFX(IntakeConstants.kIntakeMotorArm);
    m_IntakeMotorRoller = new TalonFX(IntakeConstants.kIntakeMotorRoller);

    m_IntakeState = IntakeState.S_Empty;

    // Configs that use the PID values to help with motor speed
    var m_talonFXConfigs =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(FORWARD_LIMIT)
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(REVERSE_LIMIT)
                    .withReverseSoftLimitEnable(true))
            .withHardwareLimitSwitch(
                new HardwareLimitSwitchConfigs()
                    .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin));

    var Slot0Configs = m_talonFXConfigs.Slot0;
    Slot0Configs.kP = Constants.IntakeConstants.ExtendkP;
    Slot0Configs.kI = Constants.IntakeConstants.ExtendkI;
    Slot0Configs.kD = Constants.IntakeConstants.ExtendkD;
    Slot0Configs.kS = Constants.IntakeConstants.ExtendkS;
    Slot0Configs.kV = Constants.IntakeConstants.ExtendkV;
    Slot0Configs.kA = Constants.IntakeConstants.ExtendkA;
    Slot0Configs.kG = Constants.IntakeConstants.ExtendkG;

    var Slot1Configs = m_talonFXConfigs.Slot1;
    Slot1Configs.kP = Constants.IntakeConstants.RetractkP;
    Slot1Configs.kI = Constants.IntakeConstants.RetractkI;
    Slot1Configs.kD = Constants.IntakeConstants.RetractkD;
    Slot1Configs.kS = Constants.IntakeConstants.RetractkS;
    Slot1Configs.kV = Constants.IntakeConstants.RetractkV;
    Slot1Configs.kA = Constants.IntakeConstants.RetractkA;
    Slot1Configs.kG = Constants.IntakeConstants.RetractkG;

    MotorOutputConfigs IntakeConfig = new MotorOutputConfigs();
    IntakeConfig.Inverted = InvertedValue.Clockwise_Positive;

    MotorOutputConfigs RollerIntakeConfig = new MotorOutputConfigs();
    RollerIntakeConfig.Inverted = InvertedValue.Clockwise_Positive;

    // Apply the Configs to the Motor Objects
    m_IntakeMotorArm.getConfigurator().apply(m_talonFXConfigs);
    m_IntakeMotorArm.getConfigurator().apply(IntakeConfig);
    m_IntakeMotorRoller.getConfigurator().apply(RollerIntakeConfig);

    m_IntakeMotorArm.setPosition(0);
  }

  public enum IntakeState {
    S_Extend,
    S_Retract,
    S_Outtake,
    S_Empty
  }

  public void RunIntakeState() {
    switch (m_IntakeState) {
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

  public void empty() {
    m_IntakeMotorArm.setVoltage(0);
  }

  // Gives the motor velocity using arm position
  public void setPosition(double position, int slot) {
    PositionVoltage pos = new PositionVoltage(position).withSlot(slot);
    m_IntakeMotorArm.setControl(pos);
  }

  // A method that returns true if the arm is at its destination
  public boolean checkExtended(double Setpoint) {
    double position = m_IntakeMotorArm.getPosition().getValueAsDouble();
    return (position - 0.05 <= Setpoint) && (position + 0.05 >= Setpoint); // subject to change
  }

  // Extends the intake out and starts the rollers
  public void ExtendPosition() {
    // If motor has reached its destination the stop the arm and start the rollers
    if (checkExtended(IntakeConstants.kExtensionPosition)) {
      m_IntakeMotorRoller.setVoltage(8);
    }
    // Move the arm until it reaches its destination
    setPosition(IntakeConstants.kExtensionPosition, 0);
  }

  // Pulls the intake back in and stops the rollers
  public void RetractPosition() {
    // When retracting we want to rollers to stay off
    m_IntakeMotorRoller.setVoltage(0);
    // // If the arm has reached its destination stop the motor
    // if (checkExtended(IntakeConstants.kRetractPosition)) {
    //   //m_IntakeState = IntakeState.S_Retracted;
    // }
    // Move the arm until it reaches the destination
    setPosition(IntakeConstants.kRetractPosition, 1);
  }

  // Extends the intake if it is not out and reverses the rollers to eject pieces
  public void Outtake() {
    // If motor has reached its destination the stop the arm and start the rollers moving backwards
    // (ejecting)
    if (checkExtended(IntakeConstants.kExtensionPosition)) {
      m_IntakeMotorArm.setVoltage(0);
      m_IntakeMotorRoller.setVoltage(8);
    } else {
      // Move the arm until it reaches its destination
      setPosition(IntakeConstants.kExtensionPosition, 0);
    }
  }

  @Override
  public void periodic() {
    if (m_IntakeMotorArm.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround) {
      m_IntakeMotorArm.setPosition(0);
    }
    RunIntakeState();
    SmartDashboard.putBoolean("CheckExtended", checkExtended(IntakeConstants.kExtensionPosition));
    SmartDashboard.putBoolean("CheckRetracted", checkExtended(IntakeConstants.kRetractPosition));
    SmartDashboard.putNumber("ForwardSoftLimit", FORWARD_LIMIT);
    SmartDashboard.putNumber("ReverseSoftLimit", REVERSE_LIMIT);
    SmartDashboard.putNumber("IntakePose", m_IntakeMotorArm.getPosition().getValueAsDouble());
    SmartDashboard.putString("IntakeState", m_IntakeState.toString());
    // This method will be called once per scheduler run

  }

  // A (resusable) voltage out for driving the motor during sysId
  private VoltageOut m_SysId_VoltageRequest = new VoltageOut(0);

  private void voltageCallback(Voltage voltage) {
    m_IntakeMotorArm.setControl(m_SysId_VoltageRequest.withOutput(voltage));
  }

  // Values needed for sysid logging only.
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_SysId_AppliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_SysId_Angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_SysId_Velocity = RadiansPerSecond.mutable(0);

  private void logCallback(SysIdRoutineLog log) {
    log.motor("intake")
        .voltage(
            m_SysId_AppliedVoltage.mut_replace(
                m_IntakeMotorArm.get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(
            m_SysId_Angle.mut_replace(m_IntakeMotorArm.getPosition().getValueAsDouble(), Rotations))
        .angularVelocity(
            m_SysId_Velocity.mut_replace(
                m_IntakeMotorArm.getVelocity().getValueAsDouble(), RotationsPerSecond));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_SysID.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_SysID.dynamic(direction);
  }
}
