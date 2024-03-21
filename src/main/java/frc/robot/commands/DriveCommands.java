package frc.robot.commands;

import static frc.robot.controller.IO.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveCommands {
    /** Speed Control - Drive on one joystick */
    public final Command speedDriveTest;

    /** Drive with 2 joysticks, automatically rotating toward a target AprilTag */
    public final Command visionDrive;

    /** Drive on one joystick */
    public final Command teleopDrive;

    /** Drive with two joysticks */
    public final Command teleopDrive_twoJoy;

    // TODO Test? - Might be useful for visionDrive
    /** YAGSL build in field oriented drive */
    public final Command fieldOrientedAngVel;

    public DriveCommands(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.speedDriveTest =
                new TeleopDriveSpeed(
                        driveSubsystem,
                        () -> driver.getThrottle(),
                        () -> rotation.getThrottle(),
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getZ(), IOConstants.DEADBAND),
                        () -> !driver.getRawButton(IOConstants.DRIVE_MODE_BUTTON));

        this.visionDrive =
                new TeleopDrive(
                        driveSubsystem,
                        () -> -MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> -MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () ->
                                driveSubsystem.visionTargetPIDCalc(
                                        visionSubsystem,
                                        -MathUtil.applyDeadband(
                                                rotation.getZ(), IOConstants.DEADBAND)),
                        () -> driver.getRawButton(IOConstants.DRIVE_MODE_BUTTON));

        this.teleopDrive =
                new TeleopDrive(
                        driveSubsystem,
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getZ(), IOConstants.DEADBAND),
                        () -> !driver.getRawButton(IOConstants.DRIVE_MODE_BUTTON));

        this.teleopDrive_twoJoy =
                new TeleopDrive(
                        driveSubsystem,
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(rotation.getX(), IOConstants.DEADBAND),
                        () -> !driver.getRawButton(IOConstants.DRIVE_MODE_BUTTON));

        this.fieldOrientedAngVel =
                driveSubsystem.driveCommand(
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> -MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> driver.getZ());
    }
}
