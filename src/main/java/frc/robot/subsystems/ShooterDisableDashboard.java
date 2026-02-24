// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterDisableDashboard extends SubsystemBase {
  /** Creates a new ShooterDisableDashboard. */
  private TalonFX m_LeftShooter;

  private TalonFX m_RightShooter;

  public ShooterDisableDashboard() {
    m_LeftShooter = new TalonFX(ShooterConstants.kLeftShooterId);
    m_RightShooter = new TalonFX(ShooterConstants.kRightShooterId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
