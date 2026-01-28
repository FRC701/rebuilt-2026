// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Agitator.AgitatorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AgitatorOnCommand extends InstantCommand {
  Agitator mAgitatorSubsystem;

  public AgitatorOnCommand(Agitator tempAgitator) {
    // Use addRequirements() here to declare subsystem dependencies.
    mAgitatorSubsystem = tempAgitator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mAgitatorSubsystem.mAgitatorState = AgitatorState.S_On;
  }
}
