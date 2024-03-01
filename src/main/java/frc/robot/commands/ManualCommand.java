package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ManualCommand extends Command {

    private final ArmSubsystem armSubsystem;
    private final double armSpeed;

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
        armSubsystem.stop();
    }
}
