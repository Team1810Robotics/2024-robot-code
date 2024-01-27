package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;

public class ManualCommand extends ManualCommand {
    
    
    ArmSubsystem armSubsystem;

    double movement;

    public ManualCommand(ArmSubsystem armSubsystem, double movement){
        this.movement = movement;
        this.armSubsystem = armSubsystem;

        addRequirments(armSubsystem);
    }

    public void execute(){
        armSubsystem.setSpeed(.5);
    }
    public void end(){
        armSubsystem.setSpeed(0);
    }
}