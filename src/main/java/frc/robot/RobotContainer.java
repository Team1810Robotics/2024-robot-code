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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(SwerveConstants.DIRECTORY);
    public static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public static LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final DriveCommands drive = new DriveCommands(driveSubsystem, visionSubsystem);

    private final CommandJoystick m_driver = new CommandJoystick(0);
    private final CommandJoystick m_rotationController = new CommandJoystick(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        // Create a SendableChooser to select the drive command
        SendableChooser<Command> driveChooser = new SendableChooser<>();
        driveChooser.setDefaultOption("Teleop Drive", drive.teleopDrive);
        driveChooser.addOption("Teleop Drive (Two Joysticks)", drive.teleopDrive_twoJoy);
        driveChooser.addOption("Drive Field Oriented (Needs Testing)", drive.fieldOrientedAngVel);
        driveChooser.addOption("Speed Mod Test", drive.speedDriveTest);
        Shuffleboard.getTab("Teleoperated").add("Drive Command", driveChooser);

        Command visDrive =
                new TeleopDriveVis(
                        () -> -m_driver.getThrottle(),
                        () -> -m_rotationController.getThrottle(),
                        driveSubsystem,
                        visionSubsystem,
                        m_driver.trigger(),
                        () -> MathUtil.applyDeadband(m_driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(m_driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(m_driver.getZ(), IOConstants.DEADBAND),
                        () -> true);

        Command visDrive_two =
                new TeleopDriveVis(
                        () -> m_driver.getThrottle(),
                        () -> m_rotationController.getThrottle(),
                        driveSubsystem,
                        visionSubsystem,
                        m_driver.trigger(),
                        () -> MathUtil.applyDeadband(m_driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(m_driver.getX(), IOConstants.DEADBAND),
                        () ->
                                MathUtil.applyDeadband(
                                        m_rotationController.getX(), IOConstants.DEADBAND),
                        () -> true);

        driveSubsystem.setDefaultCommand(visDrive_two);
        // driveSubsystem.setDefaultCommand(drive.teleopDrive_twoJoy);

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption(
                "5m line",
                driveSubsystem.driveToPose(new Pose2d(new Translation2d(5, 0), new Rotation2d())));
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driver_button9.onTrue(Commands.run(driveSubsystem::zeroGyro, driveSubsystem));

        driver_button12.whileTrue(new ExtenderCommand(-1, extenderSubsystem));
        driver_button10.whileTrue(new ExtenderCommand(1, extenderSubsystem));

        driver_button4.whileTrue(driveSubsystem.aimAtTarget(visionSubsystem));
        driver_button3.whileTrue(driveSubsystem.aimAtTarget(visionSubsystem));

        box_intake.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        box_outtake.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
        box_climbUp.whileTrue(
                new ClimbCommand(climbSubsystem, ClimbSubsystem.ClimbDirection.climbUp));
        box_climbDown.whileTrue(
                new ClimbCommand(climbSubsystem, ClimbSubsystem.ClimbDirection.climbDown));
        box_intakePos.whileTrue(new ShooterCommand(shooterSubsystem, intakeSubsystem, false));

        manipulatorXbox_A.onTrue(armSubsystem.setpointCommand(ArmConstants.INTAKE_POSITION));
        manipulatorXbox_Y.onTrue(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION));
        manipulatorXbox_RB.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        manipulatorXbox_LB.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        driveSubsystem.setMotorBrake(brake);
    }
}
