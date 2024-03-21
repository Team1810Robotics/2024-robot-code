package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem intake;
    private final double intakeSpeed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
        this.intake = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    public void execute() {
        intake.setSpeed(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return (!intake.hasNote() && intakeSpeed > 0);
    }
}
