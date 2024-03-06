package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class TeleopDriveSpeed extends Command{
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier rotationSpeed;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final SwerveController controller;

    public TeleopDriveSpeed(SwerveSubsystem drivetrain, DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega) {
        this.drivetrain = drivetrain;
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
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
        double speedMult = (driveSpeed.getAsDouble() + 1) / 2;  // Make axis value 0 to 1 instead of -1 to 1
        double rotationMult = (rotationSpeed.getAsDouble() + 1) / 2;
        double xVelocity = vX.getAsDouble() * speedMult;
        double yVelocity = vY.getAsDouble() * speedMult;
        double angVelocity = omega.getAsDouble() * rotationMult;

        drivetrain.drive(new Translation2d(xVelocity * Swerve.maxVelocity, yVelocity * Swerve.maxVelocity),
                         angVelocity * controller.config.maxAngularVelocity,
                         true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
