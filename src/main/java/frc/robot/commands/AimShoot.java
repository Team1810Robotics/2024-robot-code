package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;

public class AimShoot extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;

    private final BooleanSupplier blocked;
    private final boolean idle;

    private double startTime;
    private double noNoteStartTime = Double.POSITIVE_INFINITY;
    private boolean noNote_h = false;
    private double atSetpointStartTime = Double.POSITIVE_INFINITY;
    private boolean atSetpoint_h = false;

    public AimShoot(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            ArmSubsystem arm,
            VisionSubsystem vision,
            boolean idleShooter) {
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
        this.idle = idleShooter;
        blocked =
                () -> {
                    boolean isAligned = vision.isAligned() || (arm.getSetpointDegrees() == 62);
                    return !isAligned;
                };
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();

        noNoteStartTime = Double.POSITIVE_INFINITY;
        noNote_h = false;

        atSetpointStartTime = Double.POSITIVE_INFINITY;
        atSetpoint_h = false;
    }

    @Override
    public void execute() {
        boolean atSetpoint = arm.atSetpointRaw();
        shooter.setVoltage(ShooterConstants.SHOOT_SPEED);
        arm.setpoint(vision.getAngle());

        if (atSetpoint && !atSetpoint_h) {
            atSetpoint_h = true;
            atSetpointStartTime = Timer.getFPGATimestamp();
        } else if (!atSetpoint && atSetpoint_h) {
            atSetpoint_h = false;
            atSetpointStartTime = Double.POSITIVE_INFINITY;
        }

        double deltaTime = Timer.getFPGATimestamp() - startTime;
        double atSetpoint_dt = Timer.getFPGATimestamp() - atSetpointStartTime;
        if ((deltaTime >= ShooterConstants.SPIN_UP_TIME)
                && !blocked.getAsBoolean()
                && ((atSetpoint_dt >= 0.75) && atSetpoint)) intake.setSpeed(1.0);
    }

    @Override
    public boolean isFinished() {
        boolean noNote = !intake.hasNote();

        if (noNote && !noNote_h) {
            noNote_h = true;
            noNoteStartTime = Timer.getFPGATimestamp();
        }

        double noNote_dt = Timer.getFPGATimestamp() - noNoteStartTime;

        return ((noNote_dt >= 0.25) && noNote);
    }

    @Override
    public void end(boolean interupted) {
        intake.stop();
        if (idle) {
            shooter.setVoltage(Volts.of(4));
        } else {
            shooter.stop();
        }
    }
}
