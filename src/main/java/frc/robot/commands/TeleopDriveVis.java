package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
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
    private final DoubleSupplier vT;
    private final BooleanSupplier visionMode;

    public TeleopDriveVis(
            DriveSubsystem drive,
            VisionSubsystem vision,
            DoubleSupplier driveSpeed,
            DoubleSupplier rotationSpeed,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier omega,
            BooleanSupplier visionMode) {
        this.drive = drive;
        this.vision = vision;
        this.driveSpeed = driveSpeed;
        this.rotationSpeed = rotationSpeed;
        this.vX = vX;
        this.vY = vY;
        this.vT = omega;
        this.visionMode = visionMode;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Make axis value 0 to 1 instead of -1 to 1
        double speedMult = convertSpace(-driveSpeed.getAsDouble());
        double rotMult = convertSpace(-rotationSpeed.getAsDouble());

        double xVelocity = vX.getAsDouble() * speedMult * SwerveConstants.MAX_SPEED;
        double yVelocity = vY.getAsDouble() * speedMult * SwerveConstants.MAX_SPEED;
        double thetaVel_ = vT.getAsDouble() * rotMult * SwerveConstants.MAX_ANG_SPEED;

        double thetaVel = vision.visionTargetPIDCalc(thetaVel_, visionMode.getAsBoolean());

        drive.drive(new Translation2d(xVelocity, yVelocity), thetaVel, true);
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
