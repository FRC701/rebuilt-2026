// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonFX m_ArmMotor1;
  private TalonFX m_ArmMotor2;
  private static Armstate armstate;

  public Arm() {
    m_ArmMotor1 = new TalonFX(Constants.ArmConstants.kArmMotor1);
    m_ArmMotor2 = new TalonFX(Constants.ArmConstants.kArmMotor2);
  }

  public enum Armstate {
    S_Up,
    S_Down
  }

  public void runArmState() {
    switch (armstate) {
      case S_Up:
        break;
      case S_Down:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
