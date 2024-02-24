package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SwerveConstants {
        public static final double FL_CANCODER_OFFSET = 259;
        public static final double FR_CANCODER_OFFSET = 151;
        public static final double BL_CANCODER_OFFSET = 211;
        public static final double BR_CANCODER_OFFSET = 283;

        public static final int FL_CANCODER_ID = 12;
        public static final int FR_CANCODER_ID = 11;
        public static final int BL_CANCODER_ID = 10;
        public static final int BR_CANCODER_ID = 9;

        public static final int WHEEL_LOCK_TIME = 10; // in seconds

        /** Maximum speed of the robot in meters per second, used to limit acceleration. */
        public static final double MAX_SPEED = Units.feetToMeters(14.5);

        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        public static final double ANGLE_CONVERSION_FACTOR =
                SwerveMath.calculateDegreesPerSteeringRotation(12.8);

        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER
        // RESOLUTION).
        //  In this case the wheel diameter is 4 inches, which must be converted to meters to get
        // meters/second.
        //  The gear ratio is 6.75 motor revolutions per wheel rotation.
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        public static final double DRIVE_CONVERSION_FACTOR =
                SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    }

    public static final class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class ArmConstants {
        public static final int MOTOR1_ID = 0;
        public static final int MOTOR2_ID = 0;

        public static final int CANCODER_ID = 0;

        public static final double INITIAL_POSITION = 0.0;
        public static final double INTAKE_POSITION = 0;

        // TODO: tune values
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class IntakeConstants {
        public static final int MOTOR_ID = 0;
        public static final int BEAM_BREAK_PORT = 0;
    }

    public static final class ShooterConstants {
        public static final int TOP_MOTOR_ID = 0;
        public static final int BOTTOM_MOTOR_ID = 0;
    }

    public static final class ExtenderConstants {
        public static final int MOTOR_ID = 0;
        public static final int TOP_LS_PORT = 0;
        public static final int BOTTOM_LS_PORT = 0;
    }

    public static final class ClimbConstants {
        // right and left relative to the bot's perspective
        public static final int LEFT_MOTOR_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;

        public static final int LEFT_TOP_LS = 0;
        public static final int LEFT_BOTTOM_LS = 0;
        public static final int RIGHT_TOP_LS = 0;
        public static final int RIGHT_BOTTOM_LS = 0;
    }

    public static final class IOConstants {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int MANIPULATOR_XBOX_PORT = 2;

        public static final double DEADBAND = 0.3;
    }
}
