// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  /** Creates a new Aggitator. */
  public Agitator() {
    mAgitatorState = AgitatorState.S_On;
  }

  public enum AgitatorState {
    S_On,
    S_Off
  }

  public static AgitatorState mAgitatorState;

  public void runAgitatorState() {
    switch (mAgitatorState) {
      case S_On:
        break;
      case S_Off:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runAgitatorState();
  }
}
