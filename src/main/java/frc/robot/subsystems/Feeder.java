// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  public FeederState m_FeederState;

  public Feeder() {
    m_FeederState = FeederState.S_Off;
  }

  public void runFeederState() {
    switch (m_FeederState) {
      case S_On:
        break;
      case S_Off:
        break;
    }
  }

  public enum FeederState {
    S_On,
    S_Off
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    runFeederState();
  }
}
