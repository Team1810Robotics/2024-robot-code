package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    public enum LEDState {
        hasTarget(0),
        noTarget(1),
        isAligned(2),
        off(3);

        private final int value;

        LEDState(int value) {
            this.value = value;
        }

        public int get() {
            return value;
        }
    }

    private static final DigitalOutput highBit = new DigitalOutput(LEDConstants.HIGH_BIT);
    private static final DigitalOutput lowBit = new DigitalOutput(LEDConstants.LOW_BIT);

    private static LEDState state = LEDState.off;

    public static void setState(LEDState state) {
        LEDSubsystem.state = state;
    }

    @Override
    public void periodic() {
        highBit.set((state.get() & 0b10) == 0b10);
        lowBit.set((state.get() & 0b01) == 0b01);
    }
}
