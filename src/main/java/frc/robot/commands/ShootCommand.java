// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterEnumState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  Shooter m_LeftShooterSubsystem;
  Shooter m_RightShooterSubsystem;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter leftShooter, Shooter rightShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LeftShooterSubsystem = leftShooter;
    m_RightShooterSubsystem = rightShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LeftShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_Shooting;
    m_RightShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_Shooting;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_LeftShooterSubsystem.UpToSpeed();
  }
}
