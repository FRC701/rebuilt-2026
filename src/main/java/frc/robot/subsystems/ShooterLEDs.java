// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterEnumState;

public class ShooterLEDs extends SubsystemBase {
  private Candle m_Candle;
  private Shooter m_Shooter;
  private ShooterEnumState m_ShooterState;

  private final int kStartLED = 8;
  private final int kEndLED = 28;
  private final RGBWColor kNotShootingColor = new RGBWColor(new Color(255, 0, 0));
  private final RGBWColor kPassingColor = new RGBWColor(new Color(157, 0, 255)); // Vibrant Purple
  private final RGBWColor kShootingColor = new RGBWColor(new Color(0, 255, 0));
  private SolidColor m_NotShooting =
      new SolidColor(kStartLED, kEndLED).withColor(kNotShootingColor);
  private SolidColor m_Passing = new SolidColor(kStartLED, kEndLED).withColor(kPassingColor);
  private SolidColor m_Shooting = new SolidColor(kStartLED, kEndLED).withColor(kShootingColor);

  /** Creates a new ShooterLEDs. */
  public ShooterLEDs(Candle candle, Shooter shooter) {
    m_Candle = candle;
    m_Shooter = shooter;
    m_ShooterState = m_Shooter.m_ShooterEnumState;
  }

  private void UpdateLEDs() {
    ShooterEnumState shooterState = m_Shooter.m_ShooterEnumState;
    if (m_ShooterState != shooterState) {
      m_ShooterState = shooterState;
      switch (m_ShooterState) {
        case S_NotShooting:
          m_Candle.LEDs(m_NotShooting);
          break;
        case S_Passing:
          m_Candle.LEDs(m_Passing);
          break;
        case S_Shooting:
          m_Candle.LEDs(m_Shooting);
          break;
        default:
          break;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdateLEDs();
  }
}
