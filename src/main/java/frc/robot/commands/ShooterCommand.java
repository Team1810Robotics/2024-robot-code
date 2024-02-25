package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    // FIXME: both member variables should be private and final (if possible)
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setBothSpeed(shooterSpeed);
    }

    @Override
    public void end(boolean interupted) {
        // FIXME: should use stop() instead of setBothSpeed(0) because it's more clear
        shooterSubsystem.setBothSpeed(0);
    }
}
