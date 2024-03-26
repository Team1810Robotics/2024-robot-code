package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final boolean idle;
    private final boolean blocked;
    private double startTime;

    public ShooterCommand(
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            boolean idle,
            boolean blocked) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;
        this.idle = idle;
        this.blocked = blocked;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        shooter.setVoltage(ShooterConstants.SHOOT_SPEED);

        double deltaTime = Timer.getFPGATimestamp() - startTime;
        if ((deltaTime > ShooterConstants.SPIN_UP_TIME) && !blocked) intake.setSpeed(1.0);
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
