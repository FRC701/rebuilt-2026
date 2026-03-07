// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {

  Shooter m_ShooterSubsystem;

  // private Feeder m_Feeder;
  // private Timer m_Timer;

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = shooter;
    // m_Feeder = feeder;
    // addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //  m_ShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_Shooting;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_ShooterSubsystem.UpToSpeed()) {
    //   m_Feeder.m_FeederState = FeederState.S_On;
    // }

    //  else {
    //   m_Feeder.m_FeederState = FeederState.S_Off;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_ShooterSubsystem.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
    // m_Feeder.m_FeederState = FeederState.S_Off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //   if (m_ShooterSubsystem.UpToSpeed()) {
    //     m_Timer.reset();
    //     m_Timer.start();
    //     if (m_Timer.hasElapsed(3)) {
    //       return true;
    //     }
    //   } else {
    //     m_Timer.reset();
    //   }
    //   return false;
    // }
    return m_ShooterSubsystem.UpToSpeed();
  }
}
