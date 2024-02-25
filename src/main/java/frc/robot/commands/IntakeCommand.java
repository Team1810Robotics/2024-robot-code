package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    // FIXME: both member variables should be private and final (if possible)
    private IntakeSubsystem intakeSubsystem;
    private double intakeSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    public void execute() {
        intakeSubsystem.setSpeed(intakeSpeed);
        boolean beam = intakeSubsystem.getBeamBreak();

        // FIXME: this is an assignment, not a comparison. Use == instead of =
        if (beam = true) {
            intakeSubsystem.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: use stop() instead of setSpeed(0) because it's more clear
        intakeSubsystem.setSpeed(0);
    }

    /* @Override
    public boolean isFinished(){
        return (intakeSubsystem.getBeamBreak());
    } */

}
