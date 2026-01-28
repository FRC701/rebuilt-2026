// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_IntakeMotorArm;

  private TalonFX m_IntakeMotorRoller;
  private static IntakeState intakeState;

  public Intake() {
    m_IntakeMotorArm = new TalonFX(Constants.IntakeConstants.kIntakeMotorArm);
    m_IntakeMotorRoller = new TalonFX(Constants.IntakeConstants.kIntakeMotorRoller);
  }

  public enum IntakeState {
    S_Extend,
    S_Retract,
    S_Outtake
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

  //returns the Arm Position
  public double getPosition(){
    return m_IntakeMotorArm.getPosition().getValueAsDouble() / IntakeConstants.GearRatio;
  }

  //Gives the motor velocity using arm position
  public void setPosition(double position){
    PositionVoltage pos = new PositionVoltage(position * IntakeConstants.GearRatio).withSlot(0);
      m_IntakeMotorArm.setControl(pos);
  }

  public void ExtendPosition() {
    if(PointReached(IntakeConstants.kExtensionPosition)){
      m_IntakeMotorArm.setVoltage(0);
      m_IntakeMotorRoller.setVoltage(4);
    }else{
      setPosition(IntakeConstants.kExtensionPosition);
    }
  }

  public boolean PointReached(double Setpoint){
    return (getPosition() - 5 <= Setpoint) && (getPosition() + 5 >= Setpoint); //subject to change
  }

  public void RetractPosition() {
    m_IntakeMotorRoller.setVoltage(0);
    if(PointReached(IntakeConstants.kRetractPosition)){
      m_IntakeMotorArm.setVoltage(0);
    }else{
      setPosition(IntakeConstants.kRetractPosition);
    }
  }

  public void Outtake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
