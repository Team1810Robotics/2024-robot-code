package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double shooterSpeed; // CHANGE: changed name to better reflect the purpose of the variable

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double shooterSpeed){
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
        shooterSubsystem.setBothSpeed(0);
    }
}