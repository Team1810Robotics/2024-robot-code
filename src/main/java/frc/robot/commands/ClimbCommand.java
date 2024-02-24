package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{

    private ClimbSubsystem climbSubsystem;
    private double climbSpeed;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double climbSpeed) {
        this.climbSpeed = climbSpeed;
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        climbSubsystem.setMotorSpeed(climbSpeed);
        boolean rightBottom = climbSubsystem.getRightBottomlimit();
        boolean leftBottom = climbSubsystem.getRightBottomlimit();
        boolean rightTop = climbSubsystem.getRightBottomlimit();
        boolean leftTop = climbSubsystem.getRightBottomlimit();

        // FIXME: This is an infinate loop in an infinite loop, which is not good.
        while (climbSpeed != 0) {

            if(rightBottom = true){
                climbSubsystem.setMotorSpeed(0);
            } 

            else if(leftBottom = true){
                climbSubsystem.setMotorSpeed(0);
            } 

             else if(rightTop = true){
                climbSubsystem.setMotorSpeed(0);
            } 

             else if(leftTop = true){
                climbSubsystem.setMotorSpeed(0);
            } 
        }
    }
}
