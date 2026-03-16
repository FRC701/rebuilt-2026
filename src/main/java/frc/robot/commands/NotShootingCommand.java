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
public class NotShootingCommand extends InstantCommand {
  Shooter m_LeftShooterSubsystem;
  Shooter m_RightShooterSubsystem;
  Feeder m_Feeder;

  public NotShootingCommand(Shooter leftShooter, Shooter rightShooter, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LeftShooterSubsystem = leftShooter;
    m_RightShooterSubsystem = rightShooter;
    m_Feeder = feeder;
    addRequirements(m_LeftShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LeftShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
    m_RightShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
    m_Feeder.m_FeederState = FeederState.S_Off;
  }
}
