package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command  {

    private ArmSubsystem armSubsystem;
    private double armSetpoint; // CHANGE: changed name to better reflect the purpose of the variable

    public ArmCommand(ArmSubsystem armSubsystem, double armSetpoint){
        this.armSetpoint = armSetpoint;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    // CHANGE: added required @Override annotation
    @Override
    public void initialize() {
        armSubsystem.setGoal(ArmConstants.INITIAL_POSITION);
    }

    // CHANGE: added required @Override annotation
    @Override
    public void execute() {
        armSubsystem.setGoal(ArmConstants.INTAKE_POSITION);
    }

    // CHANGE: added required @Override annotation
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop(); // CHANGE: made sure to stop when command ends
    }
}


