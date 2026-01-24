// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX m_IntakeMotorArm;

  private TalonFX m_IntakeMotorRoller;
  private static IntakeState intakeState;

  public Intake() {
    m_IntakeMotorArm = new TalonFX(Constants.IntakeConstants.kIntakeMotor1);
    m_IntakeMotorRoller = new TalonFX(Constants.IntakeConstants.kIntakeMotor2);
  }

  public enum IntakeState {
    S_Up,
    S_Down,
    S_Outtake
  }

  public void RunIntakeState() {
    switch (intakeState) {
      case S_Up:
        UpPosition();
        break;
      case S_Down:
        DownPosition();
        break;
      case S_Outtake:
        Outtake();
        break;
    }
  }

  public void UpPosition() {}

  public void DownPosition() {}

  public void Outtake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
