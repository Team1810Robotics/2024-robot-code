package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class DriveCommand extends Command {
    private final DriveSubsystem drivetrain;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public DriveCommand(
            DriveSubsystem drivebase,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier omega,
            BooleanSupplier driveMode) {
        this.drivetrain = drivebase;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drivebase.getSwerveController();

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xVelocity = vX.getAsDouble();
        double yVelocity = vY.getAsDouble();
        double angVelocity = omega.getAsDouble();
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        drivetrain.drive(
                new Translation2d(xVelocity * 5, yVelocity * 5),
                angVelocity * controller.config.maxAngularVelocity,
                driveMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}
}
