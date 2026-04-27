// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterEnumState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchToggle extends InstantCommand {

  Feeder m_LeftFeeder;
  Feeder m_RightFeeder;
  Shooter m_LeftShooter;
  Shooter m_RightShooter;
  Intake m_Intake;

  public LaunchToggle(
      Feeder tempLeftFeeder,
      Feeder tempRightFeeder,
      Shooter tempLeftShooter,
      Shooter tempRightShooter,
      Intake tempIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_LeftShooter = tempLeftShooter;
    m_RightShooter = tempRightShooter;
    m_LeftFeeder = tempLeftFeeder;
    m_RightFeeder = tempRightFeeder;
    m_Intake = tempIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_LeftShooter.m_ShooterEnumState == ShooterEnumState.S_Shooting) {
      m_LeftShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_RightShooter.m_ShooterEnumState = ShooterEnumState.S_NotShooting;
      m_LeftFeeder.m_FeederState = FeederState.S_Off;
      m_RightFeeder.m_FeederState = FeederState.S_Off;
      m_Intake.m_IntakeState = IntakeState.S_ExtendCycleUp;

    } else {
      m_LeftShooter.m_ShooterEnumState = ShooterEnumState.S_Shooting;
      m_RightShooter.m_ShooterEnumState = ShooterEnumState.S_Shooting;
      m_Intake.m_IntakeState = IntakeState.S_Down;
      m_LeftFeeder.m_FeederState = FeederState.S_On;
      m_RightFeeder.m_FeederState = FeederState.S_On;
    }
  }
}
