package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class Box extends GenericHID {

    public Box(int port) {
        super(port);
    }

    public enum Button {
        // Face Arcade Buttons
        travelPos(2),
        climbPos(3),
        intakePos(4),

        // joystick
        in(9),
        up(11),
        down(10),
        out(12);

        private final int value;

        Button(int value) {
            this.value = value;
        }

        public int get() {
            return value;
        }
    }
}
