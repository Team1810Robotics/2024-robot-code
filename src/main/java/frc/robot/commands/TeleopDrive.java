package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class TeleopDrive extends Command{
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final SwerveController controller;

    public TeleopDrive(SwerveSubsystem drivetrain, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.drivetrain = drivetrain;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.controller = drivetrain.getSwerveController();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xVelocity = vX.getAsDouble();
        double yVelocity = vY.getAsDouble();
        double angVelocity = omega.getAsDouble();

        drivetrain.drive(new Translation2d(xVelocity * 5, yVelocity * 5),
                         angVelocity * controller.config.maxAngularVelocity,
                         true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
