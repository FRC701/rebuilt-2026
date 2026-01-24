// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants; 
import com.ctre.phoenix6.hardware.TalonFX; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_IntakeMotor1; 
  private TalonFX m_IntakeMotor2; 
  private static IntakeState intakeState;

  public Intake() {
    m_IntakeMotor1 = new TalonFX(Constants.IntakeConstants.kIntakeMotor1);
    m_IntakeMotor2 = new TalonFX(Constants.IntakeConstants.kIntakeMotor2);
  }
 public enum IntakeState{
  S_Rolling,
  S_Stop,
  S_Outtake
 }

 public void RunIntakeState() {
  switch (intakeState){
    case: S_Rolling:
    SpinMotor();
    break;
    case: S_Stop:
    StopMotor();
    break;
    case: S_Outtake:
    Outtake();
    break;
  }
 }

public void SpinMotor(){ 

}

public void StopMotor(){

}

public void Outtake(){

}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
