package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TeleopDriveVis extends Command {
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier rotationSpeed;
    private final CommandJoystick driver;
    private final DriveSubsystem drive;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public TeleopDriveVis(
            DoubleSupplier driveSpeed,
            DoubleSupplier rotationSpeed,
            DriveSubsystem drive,
            CommandJoystick driver,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier omega,
            BooleanSupplier driveMode) {
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
        this.drive = drive;
        this.driver = driver;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drive.getSwerveController();

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speedMult =
                (driveSpeed.getAsDouble() + 1) / 2; // Make axis value 0 to 1 instead of -1 to 1
        double rotationMult = (rotationSpeed.getAsDouble() + 1) / 2;
        double xVelocity =
                MathUtil.applyDeadband(vX.getAsDouble(), IOConstants.DEADBAND) * speedMult;
        double yVelocity =
                MathUtil.applyDeadband(vY.getAsDouble(), IOConstants.DEADBAND) * speedMult;
        double angVelocity =
                drive.visionTargetPIDCalc(
                        drive.visionSubsystem,
                        MathUtil.applyDeadband(
                                (omega.getAsDouble() * rotationMult), IOConstants.DEADBAND),
                        driver.button(1).getAsBoolean());
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        drive.drive(
                new Translation2d(xVelocity * 5, yVelocity * 5),
                angVelocity * controller.config.maxAngularVelocity,
                driveMode.getAsBoolean());
    }
}
