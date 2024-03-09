package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderCommand extends Command {

    private final ExtenderSubsystem extenderSubsystem;
    private final double speed;

    public ExtenderCommand(double speed, ExtenderSubsystem extenderSubsystem) {
        this.extenderSubsystem = extenderSubsystem;
        this.speed = speed;

        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {
        extenderSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}
