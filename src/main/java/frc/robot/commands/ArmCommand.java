package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

    private final ArmSubsystem armSubsystem;
    private final double armSetpoint;

    public ArmCommand(ArmSubsystem armSubsystem, double armSetpoint) {
        this.armSetpoint = armSetpoint;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setGoal(ArmConstants.INITIAL_POSITION);
    }

    @Override
    public void execute() {
        armSubsystem.setGoal(ArmConstants.INTAKE_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}
