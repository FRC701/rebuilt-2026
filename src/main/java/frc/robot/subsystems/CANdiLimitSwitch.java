package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdiLimitSwitch extends SubsystemBase {

  private final CANdi m_candi;
  private final StatusSignal<S1StateValue> m_signal1State;
  private final StatusSignal<S2StateValue> m_signal2State;

  public CANdiLimitSwitch(int canId, String canBus) {
    m_candi = new CANdi(canId, new CANBus(canBus));

    CANdiConfiguration candiConfig = new CANdiConfiguration();
    candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
    candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
    candiConfig.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
    m_candi.getConfigurator().apply(candiConfig);

    m_signal1State = m_candi.getS1State();
    m_signal2State = m_candi.getS2State();
  }

  public S1StateValue getSwitch1State() {
    m_signal1State.refresh();
    return m_signal1State.getValue();
  }

  public S2StateValue getSwitch2State() {
    m_signal2State.refresh();
    return m_signal2State.getValue();
  }

  public boolean isSwitch1(S1StateValue state) {
    return getSwitch1State() == state;
  }

  public boolean isSwitch2(S2StateValue state) {
    return getSwitch2State() == state;
  }

  /**
   * Example usage demonstrating how to check limit switch states.
   *
   * <p>Wiring configuration for this example: - Limit switch COM wired to GND - Limit switch NO
   * (normally open) wired to S1IN or S2IN - S1FloatState and S2FloatState set to PullHigh -
   * S1CloseState and S2CloseState set to CloseWhenLow
   *
   * <p>With this wiring: - Low: Switch is pressed (input pulled to GND through closed switch) -
   * High: Switch is released (internal pull-up pulls input high) - Floating: Switch is disconnected
   * or wiring issue
   */
  public void exampleUsage() {
    // Check if switch 1 is triggered (low)
    if (isSwitch1(S1StateValue.Low)) {
      // Limit switch 1 is pressed
    }

    // Check if switch 2 is not triggered (high)
    if (isSwitch2(S2StateValue.High)) {
      // Limit switch 2 is released
    }

    // Detect wiring issues
    if (isSwitch1(S1StateValue.Floating)) {
      // Switch 1 may be disconnected
    }

    // Use raw state with switch statement
    switch (getSwitch1State()) {
      case Low -> {
        /* pressed */ }
      case High -> {
        /* released */ }
      case Floating -> {
        /* disconnected */ }
    }
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
