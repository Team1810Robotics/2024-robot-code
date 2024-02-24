package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.io.File;

import static frc.robot.IO.*;

import frc.robot.Constants.IOConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        Command teleopDrive = new DriveCommand(
            drivebase,
            () -> -MathUtil.applyDeadband(leftJoystick.getY(), IOConstants.DEADBAND),
            () -> -MathUtil.applyDeadband(leftJoystick.getX(), IOConstants.DEADBAND),
            () -> -MathUtil.applyDeadband(leftJoystick.getZ(), 0.5),
            () -> !leftJoystick_button10.getAsBoolean()
        );

        Command teleopDrive_twoJoy = new DriveCommand(
            drivebase,
            () -> -MathUtil.applyDeadband(rightJoystick.getY(), IOConstants.DEADBAND),
            () -> -MathUtil.applyDeadband(rightJoystick.getX(), IOConstants.DEADBAND),
            () -> -MathUtil.applyDeadband(leftJoystick.getX(), 0.5),
            () -> !leftJoystick_button10.getAsBoolean()
        );

        drivebase.setDefaultCommand(teleopDrive);

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add(autoChooser);
    }

    private void configureBindings() {
        manipulatorXbox_A.onTrue(new ManualCommand(armSubsystem, 0.1)).onFalse(new ManualCommand(armSubsystem, 0));
        manipulatorXbox_B.onTrue(new ManualCommand(armSubsystem, -0.1)).onFalse(new ManualCommand(armSubsystem, 0));
        manipulatorXbox_Y.onTrue(new ClimbCommand(climbSubsystem, 0.5));
        manipulatorXbox_X.onTrue(new ClimbCommand(climbSubsystem, -0.5));

        manipulatorXbox_LB.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        manipulatorXbox_RB.whileTrue(new IntakeCommand(intakeSubsystem, -0.75));

        rightJoystick_button9.onTrue(new InstantCommand(drivebase::zeroGyro));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
