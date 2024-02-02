package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command  {

    ArmSubsystem armSubsystem;

    double arm;

    public ArmCommand(ArmSubsystem armSubsystem, double arm){
        this.arm = arm;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }
    
    public void initialize(){
        armSubsystem.setGoal(ArmConstants.INITIAL_POSITION);
    }

    public void execute(){
        armSubsystem.setGoal(ArmConstants.INTAKE_POSITION);
    }

    public void end(){
        armSubsystem.setGoal(ArmConstants.INITIAL_POSITION);
    }

    
}


