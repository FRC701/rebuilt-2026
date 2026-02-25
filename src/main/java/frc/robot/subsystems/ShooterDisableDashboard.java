// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterDisableDashboard extends SubsystemBase {
  /** Creates a new ShooterDisableDashboard. */
  public ShooterDisableDashboard() {}

  private void GetSmartDashCons() {
    SmartDashboard.putBoolean("LeftShooterStatus", ShooterConstants.kLeftShooterStatus);
    SmartDashboard.putBoolean("RightShooterStatus", ShooterConstants.kRightShooterStatus);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
