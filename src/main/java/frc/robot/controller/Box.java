package frc.robot.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class Box extends GenericHID {

    public Box(int port) {
        super(port);
    }

    public enum Button {
        // Face Arcade Buttons
        drivePos(4),
        intakePos(3),
        climb(5),

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
