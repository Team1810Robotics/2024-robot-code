package frc.robot;

import static frc.robot.IO.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(SwerveConstants.DIRECTORY);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem);

    private final DriveCommands drive = new DriveCommands(driveSubsystem, visionSubsystem);

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

        driveSubsystem.setDefaultCommand(driveChooser.getSelected());
        shooterSubsystem.setDefaultCommand(
                Commands.runOnce(
                        () -> shooterSubsystem.setSetpoint(ShooterConstants.HALF_SET_SPEED)));

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driver_button9.onTrue(new InstantCommand(driveSubsystem::zeroGyro));

        manipulatorXbox_Y.whileTrue(driveSubsystem.aimAtTarget(visionSubsystem));
        rotation_trigger.whileTrue(drive.visionDrive);

        box_intake.whileTrue(new IntakeCommand(intakeSubsystem, 1.0));
        box_outtake.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
        box_low.onTrue(
                Commands.runOnce(() -> armSubsystem.setSetpoint(ArmConstants.INTAKE_POSITION)));
        box_climb.onTrue(
                Commands.runOnce(() -> armSubsystem.setSetpoint(ArmConstants.CLIMB_POSITION)));
        box_shoot.onTrue(
                Commands.runOnce(() -> shooterSubsystem.setSetpoint(ShooterConstants.SET_SPEED)));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        driveSubsystem.setMotorBrake(brake);
    }
}
