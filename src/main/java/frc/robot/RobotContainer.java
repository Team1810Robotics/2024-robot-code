package frc.robot;

import static frc.robot.controller.IO.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    // private final ArmSubsystem armSubsystem = new ArmSubsystem();
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

        driveSubsystem.setDefaultCommand(drive.teleopDrive);

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driver_button9.onTrue(new InstantCommand(driveSubsystem::zeroGyro));

        manipulatorXbox_Y.whileTrue(driveSubsystem.aimAtTarget(visionSubsystem));
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
