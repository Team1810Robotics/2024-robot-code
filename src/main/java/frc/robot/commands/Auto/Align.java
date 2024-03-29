package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Align extends Command {

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    private boolean h;
    private double startTime = 0;

    public Align(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.drive = driveSubsystem;
        this.vision = visionSubsystem;

        addRequirements(drive, vision);
    }

    @Override
    public void initialize() {
        h = false;
    }

    @Override
    public void execute() {
        drive.drive(new Translation2d(), vision.visionTargetPIDCalc(0, true), true);
    }

    @Override
    public boolean isFinished() {
        boolean isAligned = vision.isAligned();

        if (isAligned && !h) {
            h = true;
            startTime = Timer.getFPGATimestamp();
        } else if (!isAligned && h) {
            h = false;
            startTime = Double.POSITIVE_INFINITY;
        }

        double dt = Timer.getFPGATimestamp() - startTime;
        if ((dt >= 0.5) && isAligned && h) return true;

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
