// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterEnumState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  Shooter m_ShooterSubsystem;
  Feeder m_Feeder;
  private static Timer m_Timer;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter tempShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = tempShooter;
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_Shooting;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ShooterSubsystem.VoltageCheck() + 1 >= Constants.ShooterConstants.shootRev) {
      m_Feeder.m_FeederState = FeederState.S_On;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
    m_Feeder.m_FeederState = FeederState.S_Off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ShooterSubsystem.CurrentCHeck()) {
      m_Timer.reset();
      m_Timer.start();
      if (m_Timer.hasElapsed(3)) {
        return true;
      }
    } else {
      m_Timer.reset();
    }
    return false;
  }
}
