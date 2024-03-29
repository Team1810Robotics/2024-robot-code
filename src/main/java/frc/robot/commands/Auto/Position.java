package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class Position extends Command {

    private final ArmSubsystem arm;
    private final double setpoint;

    private boolean h = false;
    private double startTime = Double.POSITIVE_INFINITY;

    public Position(ArmSubsystem armSubsystem, double setpoint) {
        this.arm = armSubsystem;
        this.setpoint = setpoint;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        h = false;
        startTime = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        arm.setpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        boolean atSetpoint = arm.atSetpointRaw();

        if (atSetpoint && !h) {
            h = true;
            startTime = Timer.getFPGATimestamp();
        } else if (!atSetpoint && h) {
            h = false;
            startTime = Double.POSITIVE_INFINITY;
        }

        double dt = Timer.getFPGATimestamp() - startTime;

        return ((dt >= 1.5) && atSetpoint);
    }
}
