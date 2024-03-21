package frc.robot;

import static frc.robot.controller.IO.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TeleopDriveVis;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(SwerveConstants.DIRECTORY);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final DriveCommands drive = new DriveCommands(driveSubsystem, visionSubsystem);

    private final CommandJoystick m_driver = new CommandJoystick(0);
    private final CommandJoystick m_rotationController = new CommandJoystick(1);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        // Create a SendableChooser to select the drive command
        SendableChooser<Command> driveChooser = new SendableChooser<>();
        driveChooser.setDefaultOption("Vision Drive", drive.visionDrive);
        driveChooser.addOption("Teleop Drive", drive.teleopDrive);
        driveChooser.addOption("Teleop Drive (Two Joysticks)", drive.teleopDrive_twoJoy);
        driveChooser.addOption("Drive Field Oriented (Needs Testing)", drive.fieldOrientedAngVel);
        driveChooser.addOption("Speed Mod Test", drive.speedDriveTest);
        Shuffleboard.getTab("Teleoperated").add("Drive Command", driveChooser);

        Command visDrive =
                new TeleopDriveVis(
                        () -> -m_driver.getThrottle(),
                        () -> -m_rotationController.getThrottle(),
                        driveSubsystem,
                        m_driver,
                        () -> m_driver.getX(),
                        () -> m_driver.getY(),
                        () -> m_driver.getZ(),
                        () -> true);

        Command visDrive_two =
                new TeleopDriveVis(
                        () -> m_driver.getThrottle(),
                        () -> m_rotationController.getThrottle(),
                        driveSubsystem,
                        m_driver,
                        () -> MathUtil.applyDeadband(m_driver.getY(), 0.05),
                        () -> MathUtil.applyDeadband(m_driver.getX(), 0.05),
                        () -> MathUtil.applyDeadband(m_rotationController.getX(), 0.05),
                        () -> true);

        driveSubsystem.setDefaultCommand(visDrive_two);
        // driveSubsystem.setDefaultCommand(drive.teleopDrive_twoJoy);

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
        Shuffleboard.getTab("vision").addDouble("jish Yaw", visionSubsystem::getYaw);
    }

    private void configureBindings() {
        driver_button9.onTrue(new InstantCommand(driveSubsystem::zeroGyro));

        driver_button12.whileTrue(new ExtenderCommand(-1, extenderSubsystem));
        driver_button10.whileTrue(new ExtenderCommand(1, extenderSubsystem));

        driver_button4.whileTrue(driveSubsystem.aimAtTarget());
        rotation_trigger.whileTrue(drive.visionDrive);

        box_intake.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        box_outtake.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
        box_climbUp.whileTrue(new ClimbCommand(climbSubsystem, ClimbDirection.climbUp));
        box_climbDown.whileTrue(new ClimbCommand(climbSubsystem, ClimbDirection.climbDown));
        box_intakePos.whileTrue(new ShooterCommand(shooterSubsystem, intakeSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        driveSubsystem.setMotorBrake(brake);
    }
}
