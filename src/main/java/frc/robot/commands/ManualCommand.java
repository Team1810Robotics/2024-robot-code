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

    public void execute(){
        armSubsystem.setSpeed(movement);
    }
    public void end(){
        armSubsystem.setSpeed(0);
    }
}