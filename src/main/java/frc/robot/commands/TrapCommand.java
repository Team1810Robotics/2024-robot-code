package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapSubsystem;

public class TrapCommand extends Command {

    private final TrapSubsystem trapsubsystem;

    private final double trapSpeed;

    public TrapCommand(TrapSubsystem trapSubsystem, double trap) {
        this.trapSpeed = trap;
        this.trapsubsystem = trapSubsystem;

        addRequirements(trapSubsystem);
    }

    public void execute() {
        trapsubsystem.setSpeed(trapSpeed);
    }

    public void end() {
        trapsubsystem.stop();
    }
}
