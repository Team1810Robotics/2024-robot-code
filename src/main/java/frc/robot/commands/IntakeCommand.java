package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private double intakeSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    public void execute(){
        intakeSubsystem.setSpeed(intakeSpeed);
        boolean beam = intakeSubsystem.getBeamBreak();
        
        if (beam = true){
            intakeSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
    }

    /* @Override
    public boolean isFinished(){
        return (intakeSubsystem.getBeamBreak());
    } */

}
