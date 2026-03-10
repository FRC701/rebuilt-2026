// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterEnumState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchToggle extends InstantCommand {

  Feeder m_Feeder;
  Shooter m_LeftShooter;
  Shooter m_RightShooter;

  public LaunchToggle(Feeder tempfeeder, Shooter tempLeftShooter, Shooter tempRightShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LeftShooter = tempLeftShooter;
    m_RightShooter = tempRightShooter;
    m_Feeder = tempfeeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_LeftShooter.m_ShooterEnumState == ShooterEnumState.S_Shooting) {
      m_LeftShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_RightShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_Feeder.m_FeederState = FeederState.S_Off;
    } else {
      m_LeftShooter.m_ShooterEnumState = ShooterEnumState.S_Shooting;
      m_RightShooter.m_ShooterEnumState = ShooterEnumState.S_Shooting;
      m_Feeder.m_FeederState = FeederState.S_On;
    }
  }
}
