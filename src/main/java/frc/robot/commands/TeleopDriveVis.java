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
    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier vT;
    private final BooleanSupplier visionMode;

    public TeleopDriveVis(
            DriveSubsystem drive,
            VisionSubsystem vision,
            DoubleSupplier vX,
            DoubleSupplier vY,
            DoubleSupplier omega,
            BooleanSupplier visionMode) {
        this.drive = drive;
        this.vision = vision;
        this.vX = vX;
        this.vY = vY;
        this.vT = omega;
        this.visionMode = visionMode;

        addRequirements(drive);
    }

    @Override
    public void execute() {

        double xVelocity = vX.getAsDouble() * SwerveConstants.MAX_SPEED;
        double yVelocity = vY.getAsDouble() * SwerveConstants.MAX_SPEED;
        double thetaVel_ = vT.getAsDouble() * SwerveConstants.MAX_ANG_SPEED;

        double thetaVel = vision.visionTargetPIDCalc(thetaVel_, visionMode.getAsBoolean());

        drive.drive(new Translation2d(xVelocity, yVelocity), thetaVel, true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0, 0), 0, false);
    }
}
