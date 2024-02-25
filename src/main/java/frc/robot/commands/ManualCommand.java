package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualCommand extends Command {

    // FIXME: both member variables should be private and final (if possible)
    private ArmSubsystem armSubsystem;
    private double armSpeed;

    public ManualCommand(ArmSubsystem armSubsystem, double armSpeed) {
        this.armSpeed = armSpeed;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setSpeed(armSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // FIXME: use stop() instead of setSpeed(0) because it's more clear
        armSubsystem.setSpeed(0);
    }
}
