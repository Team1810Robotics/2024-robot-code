package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapSubsystem;

public class TrapCommand extends Command {

    // FIXME: both member variables should be private and final (if possible)
    TrapSubsystem trapsubsystem;

    // FIXME: this name is not descriptive, would recommend using something like "trapSpeed"
    double trap;

    public TrapCommand(TrapSubsystem trapSubsystem, double trap) {
        this.trap = trap;
        this.trapsubsystem = trapSubsystem;

        addRequirements(trapSubsystem);
    }

    public void execute() {
        trapsubsystem.motorOn(trap);
    }

    public void end() {
        trapsubsystem.motorOff();
    }
}
