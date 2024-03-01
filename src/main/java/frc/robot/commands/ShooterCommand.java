package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final double shooterSpeed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setSpeed(shooterSpeed);
    }

    @Override
    public void end(boolean interupted) {
        shooterSubsystem.stop();
    }
}
