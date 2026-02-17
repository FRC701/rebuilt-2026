// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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


import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;


// The Candle subsystem interacts directly with the CANdle device.
// Other subsystems and commands can interact with this Candle Subsystem
// in order to change the state of the LEDs
// All animations are set to a specific color and with other criteria (framerate, etc), can be changed with phoenix tuner for reference 
public class CandleSubsystem extends SubsystemBase {;

  static final int kCandleID = 1;
  public CANdle m_Candle = new CANdle(kCandleID);

  /** Creates a new candle. */
  public CandleSubsystem() {
    m_Candle
        .getConfigurator()
        .apply(new LEDConfigs().withStripType(StripTypeValue.GRB).withBrightnessScalar(1.0));
        setDefaultCommand(defaultLED());
  }
   // A quite fast, purple animation, possible for shooter LEDs?
   private final LarsonAnimation m_slot0Animation = new LarsonAnimation(0, 60) 
    .withSlot(0)   
    .withColor(new RGBWColor(190, 8, 255, 0))
    .withSize(15)
    .withBounceMode(LarsonBounceValue.Front)
    .withFrameRate(Hertz.of(100)); 

    // A light, spring green color, this animation blinks slowly without fade 
  private final StrobeAnimation m_slot1Animation = new StrobeAnimation(0, 60)
    .withSlot(1)
    .withColor(new RGBWColor (70, 255, 35, 0))
    .withFrameRate(Hertz.of(1));

    // Rainbow Fade, medium speed 
  private final RgbFadeAnimation m_slot2Animation = new RgbFadeAnimation(0, 60)
    .withSlot(2)
    .withBrightness(0.541)
    .withFrameRate(Hertz.of(76.44));

    // Gold color, blinks with fade
  private final SingleFadeAnimation m_slot3Animation = new SingleFadeAnimation (0, 60)
    .withSlot (3)
    .withColor (new RGBWColor(255, 196, 7, 0))
    .withFrameRate(Hertz.of(70.952));

    // Hot pink color (correlate with our girl robot?) twinkles quite fast
  private final TwinkleAnimation m_slot4Animation = new TwinkleAnimation(0, 60)
    .withSlot(4)
    .withColor (new RGBWColor(255, 27, 51, 0))
    .withMaxLEDsOnProportion(0.78)
    .withFrameRate(Hertz.of(87.318));

    // A cool, fading rainbow animation with half speed and brightness
  private final RainbowAnimation m_slot5Animation = new RainbowAnimation(0,60)
    .withSlot(5)
    .withBrightness(0.562)
    .withDirection(AnimationDirectionValue.Forward)
    .withFrameRate(Hertz.of(51.744));

    // A fast fire animation (not even the fastest it goes :0) 
  private final FireAnimation m_slot6Animation = new FireAnimation(0, 60)
    .withSlot(6)
    .withBrightness(0.689)
    .withDirection(AnimationDirectionValue.Forward)
    .withSparking(0.504)
    .withCooling(0.159)
    .withFrameRate(Hertz.of(53.312));

    // A cherry red flow, supposed to go backwards? Doesn't give me an option to see in phoenix tuner 
  private final ColorFlowAnimation m_slot7Animation = new ColorFlowAnimation(0, 60)
    .withSlot(7)
    .withColor(new RGBWColor(255, 13, 2, 0))
    .withDirection(AnimationDirectionValue.Backward)
    .withFrameRate(Hertz.of(48.216));

    // Bright generic green color, possibly could be used for driving practice and/or robot is on or working 
  private final SolidColor[] m_colors = new SolidColor[] {
    new SolidColor(0,60).withColor(new RGBWColor(2, 255, 4, 0)),
  };
    
  
  // Calls specific animations for each robot action, needs to be called in seperate subsystems
  // for example,  m_CandleSub.shooterLED(); 

    public Command defaultLED() {
      return run(() -> {
        for (var solidColor : m_colors) {
          m_Candle.setControl(solidColor);
        }
        m_Candle.setControl(m_slot7Animation);
      });
    }

    public Command shooterLED(){
      return run(() -> {
        m_Candle.setControl(m_slot0Animation);
      });
    }

    public Command passingLED(){
      return run(() -> {
        m_Candle.setControl(m_slot3Animation);
      });
    }

    public Command intakeLED(){
      return run(() -> {
        m_Candle.setControl(m_slot1Animation);
      });
    }



    public Command climberLED(){
      return run(() -> {
        m_Candle.setControl(m_slot4Animation);
      });
    }

    public Command agitatorLED(){
      return run (() -> {
        m_Candle.setControl(m_slot6Animation);
      });
    }

    public Command feederLED(){
      return run(() -> {
        m_Candle.setControl(m_slot5Animation); 
      });
    }
    
    public Command FunLED(){
      return run(() -> {
        m_Candle.setControl(m_slot2Animation);
      });
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
