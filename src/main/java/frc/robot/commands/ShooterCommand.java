package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooter = shooterSubsystem;
        this.intake = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        intake.setSpeed(0.75);
        shooter.setSetpoint(ShooterConstants.SET_SPEED);
    }

    @Override
    public void end(boolean interupted) {
        intake.stop();
        shooter.setSetpoint(ShooterConstants.HALF_SET_SPEED);
    }
}
