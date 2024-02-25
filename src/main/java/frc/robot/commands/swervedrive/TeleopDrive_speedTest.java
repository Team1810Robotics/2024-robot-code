package frc.robot.commands.swervedrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class TeleopDrive_speedTest extends Command{
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier speed;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public TeleopDrive_speedTest(SwerveSubsystem drivetrain, DoubleSupplier speed, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.controller = drivetrain.getSwerveController();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speedMult = (speed.getAsDouble() + 1) / 2;  // Make axis value 0 to 1 instead of -1 to 1
        double xVelocity = vX.getAsDouble() * speedMult;
        double yVelocity = vY.getAsDouble() * speedMult;

        /*TODO: consider separate rotation speed multiplier from other controllers slider - Could help with manual aim
    	        Would pass in as another controller to is command as well as adding parameter above*/ 
                
        double angVelocity = omega.getAsDouble() * (speedMult * 0.8); // Makes rotation speed change slightly less than xy speeds change - Might be stupid
        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);

        drivetrain.drive(new Translation2d(xVelocity * Swerve.maxVelocity, yVelocity * Swerve.maxVelocity),
                         angVelocity * controller.config.maxAngularVelocity,
                         driveMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}
    
}
