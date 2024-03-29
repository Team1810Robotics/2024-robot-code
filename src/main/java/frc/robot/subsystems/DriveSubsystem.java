package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    /** Swerve drive object. */
    private final SwerveDrive swerveDrive;

    private final PIDController transController = new PIDController(0, 0, 0);

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public DriveSubsystem(File directory) {

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects
        // being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive =
                    new SwerveParser(directory)
                            .createSwerveDrive(
                                    SwerveConstants.MAX_SPEED,
                                    SwerveConstants.ANGLE_CONVERSION_FACTOR,
                                    SwerveConstants.DRIVE_CONVERSION_FACTOR);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Heading correction should only be used while controlling the robot via angle
        swerveDrive.setHeadingCorrection(false);
        // Disables cosine compensation for simulations since it causes discrepancies not seen in
        // real life
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        swerveDrive.pushOffsetsToControllers();

        setupPathPlanner();
        setupShuffleBoard();
    }

    /** Setup AutoBuilder for PathPlanner. */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                        // live in your Constants class
                        AutoConstants.TRANSLATION_PID,
                        // Translation PID constants
                        AutoConstants.ANGLE_PID,
                        // Rotation PID constants
                        4.5,
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent()
                            ? alliance.get() == DriverStation.Alliance.Red
                            : false;
                },
                this // Reference to this subsystem to set requirements
                );
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        zeroGyro();
        resetOdometry(new Pose2d());
        // Create the constraints to use while pathfinding
        PathConstraints constraints =
                new PathConstraints(
                        swerveDrive.getMaximumVelocity(),
                        4.0,
                        swerveDrive.getMaximumAngularVelocity(),
                        Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0); // Rotation delay distance in meters. This is how far the robot should travel
        // before attempting to rotate.
    }

    /**
     * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a
     * rotation rate, and calculates and commands module states accordingly. Can use either
     * open-loop or closed-loop velocity control for the wheel velocities. Also has field- and
     * robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
     *     in meters per second. In robot-relative mode, positive x is torwards the bow (front) and
     *     positive y is torwards port (left). In field-relative mode, positive x is away from the
     *     alliance wall (field North) and positive y is torwards the left wall when looking through
     *     the driver station glass (field West).
     * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
     *     field/robot relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        drive(translation, rotation, fieldRelative, false);
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                isOpenLoop); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void stop() {
        drive(new Translation2d(), 0, false);
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
     * when calling this method. However, if either gyro angle or module position is reset, this
     * must be called in order for odometry to keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
     * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from
     * calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestamp);
    }

    private void setupShuffleBoard() {
        Shuffleboard.getTab("swerve").add("trans", transController);
        Shuffleboard.getTab("Teleoperated").addNumber("gyro", () -> getHeading().getDegrees());

        Shuffleboard.getTab("swerve")
                .addNumber(
                        "FL Cancoder",
                        () -> swerveDrive.getModulePositions()[0].angle.getDegrees());
        Shuffleboard.getTab("swerve")
                .addNumber(
                        "FR Cancoder",
                        () -> swerveDrive.getModulePositions()[1].angle.getDegrees());
        Shuffleboard.getTab("swerve")
                .addNumber(
                        "BL Cancoder",
                        () -> swerveDrive.getModulePositions()[2].angle.getDegrees());
        Shuffleboard.getTab("swerve")
                .addNumber(
                        "BR Cancoder",
                        () -> swerveDrive.getModulePositions()[3].angle.getDegrees());

        SmartDashboard.putData(
                "Swerve Visualizer - actual",
                new Sendable() {

                    @Override
                    public void initSendable(SendableBuilder builder) {
                        // Set widget type to swerve
                        builder.setSmartDashboardType("SwerveDrive");

                        // Wheel angles for Elastic
                        builder.addDoubleProperty(
                                "Front Left Angle",
                                () -> swerveDrive.getModulePositions()[0].angle.getDegrees(),
                                null);
                        builder.addDoubleProperty(
                                "Front Right Angle",
                                () -> swerveDrive.getModulePositions()[1].angle.getDegrees(),
                                null);
                        builder.addDoubleProperty(
                                "Back Left Angle",
                                () -> swerveDrive.getModulePositions()[2].angle.getDegrees(),
                                null);
                        builder.addDoubleProperty(
                                "Back Right Angle",
                                () -> swerveDrive.getModulePositions()[3].angle.getDegrees(),
                                null);

                        // Wheel speeds for Elastic
                        builder.addDoubleProperty(
                                "Front Left Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[1],
                                null);
                        builder.addDoubleProperty(
                                "Front Right Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[3],
                                null);
                        builder.addDoubleProperty(
                                "Back Left Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[5],
                                null);
                        builder.addDoubleProperty(
                                "Back Right Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[7],
                                null);

                        // Adds rotation to the widget - Comment out to stop, fourm said it might
                        // cause issues
                        // builder.addDoubleProperty("Robot Angle", () -> gyro.getAngle(), null);

                    }
                });
        SmartDashboard.putData(
                "Swerve Visualizer - desired",
                new Sendable() {

                    @Override
                    public void initSendable(SendableBuilder builder) {
                        // Set widget type to swerve
                        builder.setSmartDashboardType("SwerveDrive");

                        // swerveDrive.get

                        // Wheel angles for Elastic
                        builder.addDoubleProperty(
                                "Front Left Angle",
                                () -> SwerveDriveTelemetry.desiredStates[0],
                                null);
                        builder.addDoubleProperty(
                                "Front Right Angle",
                                () -> SwerveDriveTelemetry.desiredStates[2],
                                null);
                        builder.addDoubleProperty(
                                "Back Left Angle",
                                () -> SwerveDriveTelemetry.desiredStates[4],
                                null);
                        builder.addDoubleProperty(
                                "Back Right Angle",
                                () -> SwerveDriveTelemetry.desiredStates[6],
                                null);

                        // Wheel speeds for Elastic
                        builder.addDoubleProperty(
                                "Front Left Velocity",
                                () -> SwerveDriveTelemetry.desiredStates[1],
                                null);
                        builder.addDoubleProperty(
                                "Front Right Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[3],
                                null);
                        builder.addDoubleProperty(
                                "Back Left Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[5],
                                null);
                        builder.addDoubleProperty(
                                "Back Right Velocity",
                                () -> SwerveDriveTelemetry.measuredStates[7],
                                null);

                        // Adds rotation to the widget - Comment out to stop, fourm said it might
                        // cause issues
                        // builder.addDoubleProperty("Robot Angle", () -> gyro.getAngle(), null);

                    }
                });
    }
}
