package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    
    IntakeSubsystem intakeSubsystem;

    double intake;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intake){
        this.intake = intake;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    public void execute(){
        intakeSubsystem.setSpeed(intake);
    }

    public void end(){
        intakeSubsystem.setSpeed(0);
    }
    public boolean isFinished(){
        if (intakeSubsystem.getBeamBreak()){
       
        return true;
        }
        return false;
    }

}