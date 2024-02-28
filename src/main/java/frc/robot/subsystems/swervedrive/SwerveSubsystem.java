package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;

import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import frc.robot.subsystems.VisionSubsystem;

public class SwerveSubsystem extends SubsystemBase
{

  private final SwerveDrive swerveDrive;

  public double maximumSpeed = Units.feetToMeters(14.5);

  public CommandJoystick driver = RobotContainer.driver;

  // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
  double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);

  // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
  double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

  public Pigeon2 gyro = new Pigeon2(13);

  PIDController rotPidController = new PIDController(0.08, 0.05, 0);

  VisionSubsystem visionSubsystem = new VisionSubsystem();
  public boolean visHasTarget = false; 

  private double deviation = 0; // How far on average are the swerve drives off target

  public SwerveSubsystem(File directory){

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
        maximumSpeed,
        angleConversionFactor,
        driveConversionFactor);

    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    setupPathPlanner();

    setupElastic();
  }

  
  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         AutoConstants.TRANSLATION_PID,
                                         // Translation PID constants
                                         AutoConstants.ANGLE_PID,
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
   );
  }

  /** Stops the drivetrain and rotates to face the best target */
  public Command aimAtTarget() {
    return run(() -> {
      if (visionSubsystem.hasTarget())
      {
        visHasTarget = true;
        drive(new Translation2d(0, 0), -visionTargetPIDCalc(0), false);
      } else {
        visHasTarget = false;
        drive(new Translation2d(0, 0), 0, false);
      }
    });
  }

  /** @return the PID output to rotate toward the best AprilTag target
   *  @param altRotation rotation speed when no target is detected
   */
  public double visionTargetPIDCalc(double altRotation){
    boolean target = visionSubsystem.hasTarget();
    double yaw = visionSubsystem.getYaw();

    //System.out.println(yaw); // Added to elastic

    if(target) {
      visHasTarget = true;
      System.out.println(yaw);
      return rotPidController.calculate(yaw);
    } else {
      visHasTarget = false;
      return altRotation;
    }
  }

  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                     );
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                                                                      translationY.getAsDouble(),
                                                                      rotation.getAsDouble() * Math.PI,
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                                          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                                        yInput,
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Finds the average average of wheel angle positions from actual and desired
   * Math could be wrong
   * 
   * @return the average deviation value
   */
  public double getAverageEncoderDivation(){
    deviation = 0;
    for (int i = 0; i < 4; i++) {
      deviation += (swerveDrive.getModulePositions()[i].angle.getDegrees() + SwerveDriveTelemetry.measuredStates[(i * 2) + 1]) / 2;
    }
    return deviation / 4;
  }

  public void setupElastic(){
    Shuffleboard.getTab("swerve").addNumber("FL Cancoder", () -> swerveDrive.getModulePositions()[0].angle.getDegrees());
    Shuffleboard.getTab("swerve").addNumber("FR Cancoder", () -> swerveDrive.getModulePositions()[1].angle.getDegrees());
    Shuffleboard.getTab("swerve").addNumber("BL Cancoder", () -> swerveDrive.getModulePositions()[2].angle.getDegrees());
    Shuffleboard.getTab("swerve").addNumber("BR Cancoder", () -> swerveDrive.getModulePositions()[3].angle.getDegrees());

    // swerveDrive.restoreInternalOffset();    // Sets encoder offsets to 0
    swerveDrive.pushOffsetsToControllers();    // Sets encoder offsets to valuse in the constants file

    //Adds a command that Pathplanner can use
    NamedCommands.registerCommand("Aim At Target", aimAtTarget()); //Call aimAtTarget from Pathplanner

    Shuffleboard.getTab("Teleoperated").add("Look at AprilTag", aimAtTarget()); // Command button that toggles aimAtTarget command.
    Shuffleboard.getTab("Teleoperated").add("Vision Rotation PID", rotPidController); // Pid for vision aim on swerve drive
    Shuffleboard.getTab("Teleoperated").addBoolean("Target In Range", () -> visHasTarget); // True/False display for if the camera can see a AprilTag
    Shuffleboard.getTab("Teleoperated").addBoolean("Target Lock", () ->
      ((visionSubsystem.getYaw() <= VisionConstants.TARGET_LOCK_RANGE) & (visionSubsystem.getYaw() >= -VisionConstants.TARGET_LOCK_RANGE)) & (visHasTarget == true)); // Show if the robot is aimed at the target within set range from Constrants

    Shuffleboard.getTab("swerve").addDouble("Vision YAW", () -> visionSubsystem.getYaw()); // Display location of one April Tag releative to the camera
    Shuffleboard.getTab("swerve").addDouble("Swerve average deviation", () -> getAverageEncoderDivation()); // Should display the average deviation for the swerve modules based on actual and desired states from YAGSL
    Shuffleboard.getTab("swerve").add("Swerve Visualizer - actual", new Sendable() { // Add a visual for what the swerve subsystem is currently doing 

        @Override
        public void initSendable(SendableBuilder builder) {
            //Set widget type to swerve
            builder.setSmartDashboardType("SwerveDrive");

            //Wheel angles for Elastic
            builder.addDoubleProperty("Front Left Angle", () ->  swerveDrive.getModulePositions()[0].angle.getDegrees(), null);
            builder.addDoubleProperty("Front Right Angle", () ->  swerveDrive.getModulePositions()[1].angle.getDegrees(), null);
            builder.addDoubleProperty("Back Left Angle", () ->  swerveDrive.getModulePositions()[2].angle.getDegrees(), null);
            builder.addDoubleProperty("Back Right Angle", () ->  swerveDrive.getModulePositions()[3].angle.getDegrees(), null);
 
            //Wheel speeds for Elastic
            builder.addDoubleProperty("Front Left Velocity", () -> SwerveDriveTelemetry.measuredStates[1], null);
            builder.addDoubleProperty("Front Right Velocity", () -> SwerveDriveTelemetry.measuredStates[3], null);
            builder.addDoubleProperty("Back Left Velocity", () -> SwerveDriveTelemetry.measuredStates[5], null);
            builder.addDoubleProperty("Back Right Velocity", () -> SwerveDriveTelemetry.measuredStates[7], null);

            //Adds rotation to the widget - Comment out to stop, fourm said it might cause issues
            //builder.addDoubleProperty("Robot Angle", () -> gyro.getAngle(), null);
        }
    });

    Shuffleboard.getTab("swerve").add("Swerve Visualizer - desired", new Sendable() { // Add a visual for what the swerve subsystem should be doing

        @Override
        public void initSendable(SendableBuilder builder) {
            //Set widget type to swerve
            builder.setSmartDashboardType("SwerveDrive");

            //swerveDrive.get

            //Wheel angles for Elastic
            builder.addDoubleProperty("Front Left Angle", () ->  SwerveDriveTelemetry.desiredStates[0], null);
            builder.addDoubleProperty("Front Right Angle", () ->  SwerveDriveTelemetry.desiredStates[2], null);
            builder.addDoubleProperty("Back Left Angle", () ->  SwerveDriveTelemetry.desiredStates[4], null);
            builder.addDoubleProperty("Back Right Angle", () ->  SwerveDriveTelemetry.desiredStates[6], null);
 
            //Wheel speeds for Elastic
            builder.addDoubleProperty("Front Left Velocity", () -> SwerveDriveTelemetry.desiredStates[1], null);
            builder.addDoubleProperty("Front Right Velocity", () -> SwerveDriveTelemetry.measuredStates[3], null);
            builder.addDoubleProperty("Back Left Velocity", () -> SwerveDriveTelemetry.measuredStates[5], null);
            builder.addDoubleProperty("Back Right Velocity", () -> SwerveDriveTelemetry.measuredStates[7], null);

            //Adds rotation to the widget - Comment out to stop, fourm said it might cause issues
            //builder.addDoubleProperty("Robot Angle", () -> gyro.getAngle(), null);
        }
    });
  }
}
