package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;

public class ClimbCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final ClimbDirection direction;

    public ClimbCommand(ClimbSubsystem climbSubsystem, ClimbDirection direction) {
        this.direction = direction;
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        switch (direction) {
            case climbUp:
                climbSubsystem.climbUp();
                break;
            case climbDown:
                climbSubsystem.climbDown();
                break;
            case climbOff:
            default:
                climbSubsystem.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }
}
