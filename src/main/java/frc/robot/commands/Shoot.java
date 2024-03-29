package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.BooleanSupplier;

public class Shoot extends Command {

    private final BooleanSupplier blocked;

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final VisionSubsystem vision;

    private double startTime;

    public Shoot(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            ArmSubsystem arm,
            VisionSubsystem vision) {
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        this.vision = vision;
        blocked =
                () -> {
                    boolean isAligned = vision.isAligned() || (arm.getSetpointDegrees() == 62);
                    return (!isAligned || !arm.atSetpoint());
                };
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        shooter.setVoltage(ShooterConstants.SHOOT_SPEED);
        arm.setpoint(vision.getAngle());

        double deltaTime = Timer.getFPGATimestamp() - startTime;
        if ((deltaTime >= ShooterConstants.SPIN_UP_TIME) && !blocked.getAsBoolean())
            intake.setSpeed(1.0);
    }

    @Override
    public void end(boolean interupted) {
        intake.stop();
        shooter.stop();
    }
}
