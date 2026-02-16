package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Wiring: Limit switch COM to GND, NO to S1IN or S2IN
public class CANdiLimitSwitch extends SubsystemBase {

    private final CANdi m_candi;
    private final StatusSignal<Boolean> m_s1Closed;
    private final StatusSignal<Boolean> m_s2Closed;

    public CANdiLimitSwitch(int canId, String canBus) {
        m_candi = new CANdi(canId, new CANBus(canBus));

        CANdiConfiguration config = new CANdiConfiguration();
        config.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh;
        config.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow;
        config.DigitalInputs.S2FloatState = S2FloatStateValue.PullHigh;
        config.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;
        m_candi.getConfigurator().apply(config);

        m_s1Closed = m_candi.getS1Closed();
        m_s2Closed = m_candi.getS2Closed();
    }

    public boolean isSwitch1Pressed() {
        return m_s1Closed.refresh().getValue();
    }

    public boolean isSwitch2Pressed() {
        return m_s2Closed.refresh().getValue();
    }

    public CANdi getCANdi() {
        return m_candi;
    }
}