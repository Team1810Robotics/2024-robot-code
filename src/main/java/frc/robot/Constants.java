package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * For Example:
 * <pre>{@code import static frc.robot.Constants.*; }</pre>
 */
public final class Constants {
    // CHANGE: remove drive constants

    public static final class ArmConstants {
        public static final int MOTOR1_PORT = 0;
        public static final int MOTOR2_PORT = 0;
        public static final int CANCODER_PORT = 0; // CHANGE: add port number

        public static final double INITIAL_POSITION = 0.0;
        public static final double INTAKE_POSITION = 0;

        //TODO: tune values
        public static final double kP = 0.0; 
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final Constraints CONSTRAINTS
            = new Constraints(0, 0);
    }

    public static final class IntakeConstants {
        public static final int MOTOR_PORT = 2;
        public static final int BEAM_BREAK_PORT = 10;
    }

    public static final class ShooterConstants {
        public static final int TOP_MOTOR_PORT = 4;
        public static final int BOTTOM_MOTOR_PORT = 1;
    }

    public static final class ExtenderConstants {
        public static final int MOTOR_PORT = 7;
        public static final int TOP_LS_PORT = 6;
        public static final int BOTTOM_LS_PORT = 5;
    }

    public static final class ClimbConstants {
        public static final int LEFT_MOTOR_PORT = 9;
        public static final int RIGHT_MOTOR_PORT = 8;

        public static final int LEFT_TOP_LS = 0;
        public static final int LEFT_BOTTOM_LS = 0;
        public static final int RIGHT_TOP_LS = 0;
        public static final int RIGHT_BOTTOM_LS = 0;
    }

    // CHANGE: added OI constants for controllers
    public static final class OIConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int MANIPULATOR_XBOX_PORT = 1;

        public static final double DEADBAND = 0.1; // TODO: tune deadband
    }
}