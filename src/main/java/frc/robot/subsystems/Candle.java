// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.RGBWColor;


// The Candle subsystem interacts directly with the CANdle device.
// Other subsystems and commands can interact with this Candle Subsystem
// in order to change the state of the LEDs
public class Candle extends SubsystemBase {

  static final int kCandleID = 1;
  private CANdle m_Candle = new CANdle(kCandleID);

  /** Creates a new candle. */
  public Candle() {
    m_Candle
        .getConfigurator()
        .apply(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0));
  }

    private final SolidColor m_slot0Animation = new SolidColor(0, 60)
      .withLEDStartIndex(kCandleID)
      .withColor(new RGBWColor( 34, 139, 34, 0))
      .withMaxLEDsOnProportion(0.5)
      .withFrameRate(Hertz.of(10));

    
  }

  void LEDs(EmptyAnimation leds) {
    m_Candle.setControl(leds);
  }

  void LEDs(ColorFlowAnimation leds) {
    m_Candle.setControl(leds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
