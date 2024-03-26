package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.controller.IO.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;

public class RobotContainer {

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(SwerveConstants.DIRECTORY);
    public static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public static LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        Command visDrive =
                new TeleopDriveVis(
                        driveSubsystem,
                        visionSubsystem,
                        () -> -driver.getThrottle(),
                        () -> -driver.getThrottle(),
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getZ(), IOConstants.DEADBAND),
                        () -> driver.getTrigger(),
                        () -> true);

        Command visDrive_two =
                new TeleopDriveVis(
                        driveSubsystem,
                        visionSubsystem,
                        () -> driver.getThrottle(),
                        () -> rotation.getThrottle(),
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(rotation.getX(), IOConstants.DEADBAND),
                        () -> driver.getTrigger(),
                        () -> true);

        driveSubsystem.setDefaultCommand(visDrive_two);

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption(
                "5m line",
                driveSubsystem.driveToPose(new Pose2d(new Translation2d(5, 0), new Rotation2d())));
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driver_button9.onTrue(Commands.run(driveSubsystem::zeroGyro, driveSubsystem));

        driver_trigger.whileTrue(
                new Shoot(shooterSubsystem, intakeSubsystem, armSubsystem, visionSubsystem));

        box_intake.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        box_outtake.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
        box_climbUp.whileTrue(new ClimbCommand(climbSubsystem, ClimbDirection.climbUp));
        box_climbDown.whileTrue(new ClimbCommand(climbSubsystem, ClimbDirection.climbDown));
        box_intakePos.onTrue(armSubsystem.setpointCommand(ArmConstants.INTAKE_POSITION));
        box_travelPos.onTrue(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        driveSubsystem.setMotorBrake(brake);
    }
}
