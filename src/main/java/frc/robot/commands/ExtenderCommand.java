package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderCommand extends Command {

    private final ExtenderSubsystem extender;
    private final boolean forward;

    public ExtenderCommand(boolean forward, ExtenderSubsystem extenderSubsystem) {
        this.extender = extenderSubsystem;
        this.forward = forward;

        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {
        if (forward) extender.forward();
        else extender.reverse();
    }

    @Override
    public void end(boolean interrupted) {
        extender.off();
    }
}
