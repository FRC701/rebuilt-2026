// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX mClimberMotor1; 
  private TalonFX mClimberMotor2; 

  public static ClimberState mClimberState;

  public enum ClimberState{
    S_Default, S_top, S_Extend, S_Lock, S_Reset
  }
  public Climber() {
    mClimberMotor1 = new TalonFX(Constants.ClimberConstants.kClimberMotor1);
    mClimberMotor2 = new TalonFX(Constants.ClimberConstants.kClimberMotor2);
  }
  
  public void runClimberState(){
   switch (mClimberState){
    case S_Default:
    break;
    case S_top:
    break;
    case S_Extend:
    break;
    case S_Lock:
    break;
    case S_Reset:
    break;
    }
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
