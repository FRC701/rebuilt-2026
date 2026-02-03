// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederOnAndOffCommand extends InstantCommand {
  Feeder m_FeederSubsystem;

  public FeederOnAndOffCommand(Feeder tempFeeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_FeederSubsystem = tempFeeder;
  }

  // If the feeder is off, the command turns it on, and vice versa
  @Override
  public void initialize() {
    if (m_FeederSubsystem.m_FeederState == FeederState.S_Off) {
      m_FeederSubsystem.m_FeederState = FeederState.S_On;
    } else {
      m_FeederSubsystem.m_FeederState = FeederState.S_Off;
    }
  }
}
