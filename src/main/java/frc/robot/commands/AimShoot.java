package frc.robot.commands;

import static frc.robot.controller.IO.driver_button2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.controller.IO;
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
    private boolean idle;

    private double startTime;
    private double noNoteStartTime = Double.POSITIVE_INFINITY;
    private boolean noNote_h = false;
    private double atSetpointStartTime = Double.POSITIVE_INFINITY;
    private boolean atSetpoint_h = false;
    private IO io;

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
                    boolean isAligned =
                            vision.isAligned()
                                    || (arm.getSetpointDegrees()
                                            == ArmConstants.CLOSE_SHOOT_POSITION);
                    return !isAligned;
                };

        // Shuffleboard.getTab("intake").addBoolean("Blocked", blocked);
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

        if (driver_button2.getAsBoolean()) {
            intake.setSpeed(1);
        } else {
            intake.setSpeed(0);
        }

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
        if (idle && intake.hasNote()) {
            shooter.setVoltage(ShooterConstants.IDLE_SPEED);
        } else {
            shooter.stop();
        }
    }
}
