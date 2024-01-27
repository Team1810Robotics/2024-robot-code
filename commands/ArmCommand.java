package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandsBase  {

    ArmSubsystem armSubsystem;

    double arm;

    public ArmCommand(ArmSubsystem armSubsystem, double arm){
        this.arm = arm;
        this.armSubsystem = armSubsystem;

        addRequirments(armSubsystem);
    }
    
    public void initialize(){
        ArmSubsystem.setgoal(ArmConstants.INITIAL_POSITION);
    }

    public void execute(){
        ArmSubsystem.setgoal(ArmConstants.INTAKE_POSITION);
    }

    public void end(){
        ArmSubsystem.setgoal(ArmConstants.INITIAL_POSITION);
    }
}


