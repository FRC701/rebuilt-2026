// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialShoot extends SequentialCommandGroup {
  /** Creates a new SequentialShoot. */
  Shooter m_LeftShooter;

  Shooter m_RightShooter;
  Feeder m_LeftFeeder;
  Feeder m_RightFeeder;

  public SequentialShoot(
      Shooter leftShooter, Shooter rightShooter, Feeder leftFeeder, Feeder rightFeeder) {
    m_LeftShooter = leftShooter;
    m_RightShooter = rightShooter;
    m_LeftFeeder = leftFeeder;
    m_RightFeeder = rightFeeder;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootingCommand(m_LeftShooter, m_RightShooter),
        new ShootCommand(m_LeftShooter),
        new FeederOn(m_LeftFeeder, m_RightFeeder));
  }
}
