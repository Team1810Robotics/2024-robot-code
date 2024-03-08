package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveController;

@SuppressWarnings("unused")
public class OmniDrive extends Command{
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier rotationSpeed;
    private final DoubleSupplier dX;
    private final DoubleSupplier dY;
    private final DoubleSupplier dZ;
    private final DoubleSupplier rY;
    private final SwerveController controller;

    private CommandJoystick driver;
    private CommandJoystick rotation;

    private GenericEntry dualJoystick;
    private VisionSubsystem visionSubsystem;

    public OmniDrive(GenericEntry dualJoystick, VisionSubsystem visionSubsystem, CommandJoystick rotation, CommandJoystick driver, SwerveSubsystem drivetrain, DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed, DoubleSupplier dX, DoubleSupplier dY, DoubleSupplier dZ, DoubleSupplier rY) {
        this.drivetrain = drivetrain;
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
        this.dualJoystick = dualJoystick;
        this.rotation = rotation;
        this.driver = driver;
        this.dX = dX;
        this.dY = dY;
        this.dZ = dZ;
        this.rY = rY;
        this.controller = drivetrain.getSwerveController();
        this.visionSubsystem = visionSubsystem;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speedMult = (driveSpeed.getAsDouble() + 1) / 2;  // Make axis value 0 to 1 instead of -1 to 1
        double rotationMult = (rotationSpeed.getAsDouble() + 1) / 2;
        double xVelocity = dX.getAsDouble() * speedMult;
        double yVelocity = dY.getAsDouble() * speedMult;
        double driverRotation = dZ.getAsDouble();
        double rotationRotation = rY.getAsDouble();
        double angVelocity;

    if (dualJoystick.getBoolean(true) == true){
        angVelocity = drivetrain.visionTargetPIDCalc(
                                        visionSubsystem,
                                        MathUtil.applyDeadband(
                                                rotationRotation, IOConstants.DEADBAND),
                                        driver.button(1).getAsBoolean());
    }   else{
            angVelocity = drivetrain.visionTargetPIDCalc(
                                        visionSubsystem,
                                        MathUtil.applyDeadband(
                                                driverRotation, IOConstants.DEADBAND),
                                        driver.button(1).getAsBoolean());

        }
        drivetrain.drive(new Translation2d(xVelocity * Swerve.maxVelocity, yVelocity * Swerve.maxVelocity),
                         (angVelocity * controller.config.maxAngularVelocity) * rotationMult,
                         true);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {return false;}

}
