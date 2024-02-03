package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualCommand extends Command {
    
    
    ArmSubsystem armSubsystem;

    double movement;

    public ManualCommand(ArmSubsystem armSubsystem, double movement){
        this.movement = movement;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }
    @Override
    public void execute(){
        armSubsystem.setSpeed(movement);
    }
    @Override 
    public void end(boolean interrupted){
        armSubsystem.setSpeed(0);
    }
}