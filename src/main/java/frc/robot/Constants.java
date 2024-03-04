package frc.robot;

import java.io.File;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public final class Constants
{

  public static class IOConstants {
        public static final double DEADBAND = 0.3;
        public static final int driveSpeedModAxis = 4;
        public static final int angleSpeedModAxis = 4;
        public static final int driveXAxis = 1;
        public static final int driveYAxis = 0;
        public static final int driveOmegaAxis = 2;
        public static final int resetGyroButton = 9;
        public static final int driveModeButton = 7; // I think we want it to always be Field Oriented - At least from when I last asked Evan M

        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int MANIPULATOR_XBOX_PORT = 2;
    }

    public static class Swerve {
        public static final File directory = new File(Filesystem.getDeployDirectory(), "swerve/falcon"); //Falcons or Neos setting
        public static final double maxVelocity = 4.5; //Neo L2 max speed - I think
        public static final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1024);
        public static final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    }

    public static final class VisionConstants {
      public static Transform3d CAMERA_OFFSET = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));

      public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

      //How far off can the Robot Yaw be from the Vision Yaw
      //Used to check if the aim is ready to shoot note - Led? - Elastic Go-No-Go
      public static final double TARGET_LOCK_RANGE = 0.5;

      public static final PIDConstants VISION_PID = new PIDConstants(0.15, 0.32, 0.006);

  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutoConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
