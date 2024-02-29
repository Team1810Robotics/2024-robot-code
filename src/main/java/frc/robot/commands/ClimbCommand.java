package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final double climbSpeed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double climbSpeed) {
        this.climbSpeed = climbSpeed;
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        climbSubsystem.setSpeed(climbSpeed);
        boolean rightBottom = climbSubsystem.getRightBottomlimit();
        boolean leftBottom = climbSubsystem.getRightBottomlimit();
        boolean rightTop = climbSubsystem.getRightBottomlimit();
        boolean leftTop = climbSubsystem.getRightBottomlimit();
       
            if (rightBottom == true && climbSpeed != 0) {
                climbSubsystem.setSpeed(0);
            } else if (leftBottom == true && climbSpeed != 0) {
                climbSubsystem.setSpeed(0);
            } else if (rightTop == true && climbSpeed != 0) {
                climbSubsystem.setSpeed(0);
            } else if (leftTop == true && climbSpeed != 0) {
                climbSubsystem.setSpeed(0);
            }
        }
    }
