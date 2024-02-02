package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterCommand extends Command{
    
    
    ShooterSubsystem shooterSubsystem;

    double launch;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double launch){
        this.launch = launch;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    public void execute(){
       shooterSubsystem.setBothSpeed(launch);
    }

    public void end(){
        shooterSubsystem.setBothSpeed(0);
    }
    
}