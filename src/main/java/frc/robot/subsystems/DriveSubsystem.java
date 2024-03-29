package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    /** Swerve drive object. */
    private final SwerveDrive swerveDrive;

    private final PIDController rotController = new PIDController(0.15, 0.32, 0.006);
    private final PIDController transController = new PIDController(0, 0, 0);

    PIDController rotPidController =
            new PIDController(VisionConstants.V_Kp, VisionConstants.V_Ki, VisionConstants.V_Kd);

    public VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

    public boolean visHasTarget = false;

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

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public DriveSubsystem(
            SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveConstants.MAX_SPEED);
    }

    /** Setup AutoBuilder for PathPlanner. */
    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto
                // has a starting pose)
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

    /** Stops the drivetrain and rotates to face the best target */
    public Command aimAtTarget() {
        return run(
                () -> {
                    if (visionSubsystem.hasTarget()) {
                        drive(
                                new Translation2d(0, 0),
                                -visionTargetPIDCalc(visionSubsystem, 0, true),
                                true);
                    } else {
                        drive(new Translation2d(0, 0), 0, true);
                    }
                });
    }

    /**
     * @return the PID output to rotate toward the best AprilTag target
     * @param altRotation rotation speed when no target is detected
     */
    public double visionTargetPIDCalc(
            VisionSubsystem vision, double altRotation, boolean visionMode) {
        boolean target = vision.hasTarget();
        Optional<Double> yaw = vision.getYaw();

        if (target && visionMode && yaw.isPresent()) {
            return rotPidController.calculate(yaw.get());
        }
        if ((visionMode == true) && !target) {
            return altRotation;
        }
        return altRotation;
    }

    /**
     * @return the PID output to rotate toward the best AprilTag target
     * @param altRotation rotation speed when no target is detected
     */
    public double visionTargetPIDCalc(VisionSubsystem vision, double altRotation) {
        boolean target = vision.hasTarget();
        var yaw = vision.getYaw();

        if (yaw.isEmpty()) {
            return altRotation;
        }

        if (target) {
            return rotController.calculate(yaw.get());
        }
        return altRotation;
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(pathName);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
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

    public Command driveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(
                () -> {
                    // Make the robot move
                    swerveDrive.drive(
                            new Translation2d(
                                    Math.pow(translationX.getAsDouble(), 3)
                                            * swerveDrive.getMaximumVelocity(),
                                    Math.pow(translationY.getAsDouble(), 3)
                                            * swerveDrive.getMaximumVelocity()),
                            Math.pow(angularRotationX.getAsDouble(), 3)
                                    * swerveDrive.getMaximumAngularVelocity(),
                            true,
                            false);
                });
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
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
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
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
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
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
     * direction. The other for the angle of the robot.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(
            double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(
                xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                SwerveConstants.MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the
     * robot at an offset of 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(
                xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                SwerveConstants.MAX_SPEED);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /** Lock the swerve drive to prevent it from moving. */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestamp);
    }

    /** Add a fake vision reading for testing purposes. */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(
                new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    private void setupShuffleBoard() {

        Shuffleboard.getTab("swerve").add("hehw", rotPidController);

        Shuffleboard.getTab("swerve").add("rot", rotController);
        Shuffleboard.getTab("swerve").add("trans", transController);

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
