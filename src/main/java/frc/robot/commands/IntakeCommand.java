package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;

    private double intake;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intake){
        this.intake = intake;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.setSpeed(intake);
    }

    @Override
    public void end(boolean i){
        intakeSubsystem.setSpeed(0);
    }

    /* @Override
    public boolean isFinished(){
        return (intakeSubsystem.getBeamBreak());
    } */

}