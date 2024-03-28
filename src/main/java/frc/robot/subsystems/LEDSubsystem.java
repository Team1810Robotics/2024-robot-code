package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private static final DigitalOutput highBit = new DigitalOutput(LEDConstants.HIGH_BIT);
    private static final DigitalOutput lowBit = new DigitalOutput(LEDConstants.LOW_BIT);

    private static LEDState state = LEDState.off;

    public enum LEDState {
        hasTarget(0b00),
        noTarget(0b01),
        isAligned(0b10),
        off(0b11);

        private final int value;

        LEDState(int value) {
            this.value = value;
        }

        public int get() {
            return value;
        }
    }

    public static void setState(LEDState state) {
        LEDSubsystem.state = state;
    }

    @Override
    public void periodic() {
        boolean highBitSet = (state.get() & 0b10) == 0b10;
        boolean lowBitSet = (state.get() & 0b01) == 0b01;
        highBit.set(highBitSet);
        lowBit.set(lowBitSet);
    }
}
