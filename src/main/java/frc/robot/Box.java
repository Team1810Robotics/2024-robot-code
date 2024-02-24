package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class Box extends GenericHID {

    public Box(int port) {
        super(port);
    }

    public enum Button {
        // Face Arcade Buttons
        climb(2),
        low(3),
        autoTarget(4),
        trimUp(5),
        trimDown(6),

        // joystick
        in(9),
        shoot(10),
        unused(11),
        out(12);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }
}