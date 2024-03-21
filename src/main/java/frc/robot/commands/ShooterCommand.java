package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private double startTime;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        shooter.setSpeed(ShooterConstants.SHOOT_SPEED);

        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime > ShooterConstants.SPIN_UP_TIME) intake.setSpeed(1.0);
    }

    @Override
    public void end(boolean interupted) {
        intake.stop();
        shooter.setSpeed(4);
    }
}
