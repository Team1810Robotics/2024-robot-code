package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    private double launch;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double launch){
        this.launch = launch;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
       shooterSubsystem.setBothSpeed(launch);
    }

    @Override
    public void end(boolean interupted){
        shooterSubsystem.setBothSpeed(0);
    }
    
}