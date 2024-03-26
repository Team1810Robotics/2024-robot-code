package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveVis extends Command {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final DoubleSupplier driveSpeed;
    private final DoubleSupplier rotationSpeed;
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier visionMode;
    private final BooleanSupplier driveMode;

    public TeleopDriveVis(
            DriveSubsystem drive,
            VisionSubsystem vision,
            DoubleSupplier driveSpeed,
            DoubleSupplier rotationSpeed,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier omega,
            BooleanSupplier visionMode,
            BooleanSupplier driveMode) {
        this.drive = drive;
        this.vision = vision;
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.visionMode = visionMode;
        this.driveMode = driveMode;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Make axis value 0 to 1 instead of -1 to 1
        double speedMult = convertSpace(-driveSpeed.getAsDouble());
        double rotationMult = convertSpace(-rotationSpeed.getAsDouble());
        double xVelocity =
                MathUtil.applyDeadband(vX.getAsDouble(), IOConstants.DEADBAND) * speedMult;
        double yVelocity =
                MathUtil.applyDeadband(vY.getAsDouble(), IOConstants.DEADBAND) * speedMult;
        double controllerSpeed =
                MathUtil.applyDeadband((-omega.getAsDouble()), IOConstants.DEADBAND)
                        * rotationMult
                        * SwerveConstants.MAX_ANG_SPEED;
        double angVelocity =
                drive.visionTargetPIDCalc(vision, controllerSpeed, visionMode.getAsBoolean());

        SmartDashboard.putNumber("vX", xVelocity);
        SmartDashboard.putNumber("vY", yVelocity);
        SmartDashboard.putNumber("omega", angVelocity);
        SmartDashboard.putBoolean("Target Button", visionMode.getAsBoolean());

        drive.drive(
                new Translation2d(
                        xVelocity * SwerveConstants.MAX_SPEED,
                        yVelocity * SwerveConstants.MAX_SPEED),
                angVelocity,
                driveMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false);
    }

    /**
     * Converts from [-1, 1] to [0, 1] space
     *
     * @param value value in [-1, 1] space to convert
     * @return value converted to [0, 1] space
     */
    private double convertSpace(double value) {
        return (value + 1) / 2;
    }
}
