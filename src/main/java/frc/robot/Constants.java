package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
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

    public static class IOConstants {
        public static final double DEADBAND = 0.05;
        // Drive Controller Speed
        public static final int DRIVE_MODE_BUTTON = 7;

        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int XBOX_PORT = 2;
        public static final int BOX_PORT = 3;
    }

    public static class SwerveConstants {
        public static final File DIRECTORY =
                new File(Filesystem.getDeployDirectory(), "swerve/neo");
        public static final double maxVelocity = 5;
        public static final double driveConversionFactor =
                SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1024);
        public static final double angleConversionFactor =
                SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);

        public static final double FL_CANCODER_OFFSET = 283.271;
        public static final double FR_CANCODER_OFFSET = 34.5410;
        public static final double BL_CANCODER_OFFSET = 102.480;
        public static final double BR_CANCODER_OFFSET = 261.738;

        public static final int FL_CANCODER_ID = 9;
        public static final int FR_CANCODER_ID = 10;
        public static final int BL_CANCODER_ID = 11;
        public static final int BR_CANCODER_ID = 12;

        public static final int WHEEL_LOCK_TIME = 10; // in seconds

        public static final double MODULE_OFFSET_FROM_CENTER = 9.125;
        public static final double MAX_SPEED = 4.4196;
        public static final double MAX_ANG_SPEED =
                SwerveMath.calculateMaxAngularVelocity(
                        MAX_SPEED, MODULE_OFFSET_FROM_CENTER, MODULE_OFFSET_FROM_CENTER);

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

    public static final class VisionConstants {
        public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

        public static final double CAMERA_HEIGHT = 0.0;
        public static final double APRILTAG_RED_SHOOTER_HEIGHT = 0.0;
        public static final double CAMERA_PITCH = 0.0;

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final Transform3d CAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

        // How off can the AprilTag be and still alright
        // Used to check if the aim is ready to shoot note - Led? - Elastic Go-No-Go
        public static final double TARGET_LOCK_RANGE = 0.5;

        public static Transform3d CAMERA_OFFSET =
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

        public static final String TARGET_CAMERA = "Arducam_OV9281_USB_Camera";

        public static final double V_Kp = 0.023;
        public static final double V_Ki = 0.32;
        public static final double V_Kd = 0.006;

        public static final int APRILTAG_SPEAKER_CENTER_BLUE = 7;
        public static final int APRILTAG_SPEAKER_CENTER_RED = 4;
    }

    public static final class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class ArmConstants {

        public static final double MAX_SPEED = Math.PI / 4.0; // rad/s
        public static final double MAX_ACCEL = MAX_SPEED / 2.0; // rad/s^2

        public static final TrapezoidProfile.Constraints CONSTRAINTS =
                new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL);

        public static final int MOTOR_A_ID = 17;
        public static final int MOTOR_B_ID = 18;

        public static final int CANCODER_ID = 23;
        public static final double CANCODER_OFFSET = -1.667437116431692; // radians

        public static final double INITIAL_POSITION = 90.0;
        public static final double INTAKE_POSITION = 43.0;
        public static final double DRIVE_POSITION = 55.0;

        public static final double SETPOINT_OFFSET = 30; // degrees

        public static final double kP = 9.0;
        public static final double kI = 0.01;
        public static final double kD = 0.0;
        public static final double kIZ = 0.01;

        public static final double ks = 0.1;
        public static final double kg = 0.4;
        public static final double kv = -2.5;

        public static final double TICKS_TO_RAD_CONVERSION = (2 * Math.PI);
    }

    public static final class IntakeConstants {
        public static final int MOTOR_ID = 16;
        public static final int BEAM_BREAK_PORT = 0;
    }

    public static final class ShooterConstants {
        public static final int TOP_MOTOR_ID = 14;
        public static final int BOTTOM_MOTOR_ID = 15;
        public static final Measure<Voltage> SHOOT_SPEED = Volts.of(12.0); // Volts
        public static final double SPIN_UP_TIME = 0.5; // seconds
    }

    public static final class ExtenderConstants {
        public static final int MOTOR_ID = 22;
    }

    public static final class ClimbConstants {
        // right and left relative to the bot's perspective - forward being shooter side
        public static final int LEFT_MOTOR_ID = 19;
        public static final int RIGHT_MOTOR_ID = 20;

        public static final int LEFT_TOP_LS = 1;
        public static final int LEFT_BOTTOM_LS = 2;
        public static final int RIGHT_TOP_LS = 3;
        public static final int RIGHT_BOTTOM_LS = 5;
        public static final double CLIMB_SPEED = 0.75;
    }

    public static final class TrapConstants {
        public static final int VICTOR_MOTOR_ID = 0;
    }

    public static final class LEDConstants {
        public static final int HIGH_BIT = 8; // DIO ports
        public static final int LOW_BIT = 9; // DIO ports
    }
}
