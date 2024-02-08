package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.SwerveMath;

public class Constants {
    public static final class DriveConstants {
        public static final int PIGEON = 13;
    }

    public static class IO {
        public static final double swerveDeadband = 0.3;
        public static final int driveXAxis = 1;
        public static final int driveYAxis = 0;
        public static final int driveOmegaAxis = 4; //2 on joystick
        public static final int resetGyroButton = 9;
        public static final int driveModeButton = 7;
    }

    public static class Swerve {
        public static final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
        public static final double maxVelocity = 5;
        public static final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1024);
        public static final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    }
}
