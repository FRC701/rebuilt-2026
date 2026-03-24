// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Agitator.AgitatorState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterEnumState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberUpDownToggle extends InstantCommand {
  Climber m_Climber;
  Agitator m_Agitator;
  Feeder m_Feeder;
  Shooter m_LeftShooter;
  Shooter m_RightShooter;

  public ClimberUpDownToggle(
      Climber climber,
      Agitator agitator,
      Feeder feeder,
      Shooter leftShooter,
      Shooter rightShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climber = climber;
    m_Agitator = agitator;
    m_Feeder = feeder;
    m_LeftShooter = leftShooter;
    m_RightShooter = rightShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_Climber.m_ClimberState == ClimberState.S_Retract) {
      m_Agitator.m_AgitatorState = AgitatorState.S_Off;
      m_Feeder.m_FeederState = FeederState.S_Off;
      m_LeftShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_RightShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_Climber.m_ClimberState = ClimberState.S_Extend;
    } else {
      m_Climber.m_ClimberState = ClimberState.S_Retract;
    }
  }
}
