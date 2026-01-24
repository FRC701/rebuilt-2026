// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  /** Creates a new Aggitator. */
  private TalonFX AgitatorMotor;

  public Agitator() {
    AgitatorMotor = new TalonFX(0);
    mAgitatorState = AgitatorState.S_On;
  }

  public void spinAgitatorMotor() {}

  public void stopAgitatorMotor() {}

  public enum AgitatorState {
    S_On,
    S_Off
  }

  public AgitatorState mAgitatorState;

  public void runAgitatorState() {
    switch (mAgitatorState) {
      case S_On:
        spinAgitatorMotor();
        break;
      case S_Off:
        stopAgitatorMotor();
        break;
    }
  }

  @Override
  public void periodic() {
    runAgitatorState();
  }
}
