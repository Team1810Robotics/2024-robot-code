package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command{

    ClimbSubsystem climbSubsystem;

    double climb;

    public ClimbCommand(ClimbSubsystem climbSubsystem, double climb){
        this.climb = climb;
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute(){
        climbSubsystem.setMotorSpeed(climb);
        boolean rightBottom = climbSubsystem.getRightBottomlimit();
        boolean leftBottom = climbSubsystem.getRightBottomlimit();
        boolean rightTop = climbSubsystem.getRightBottomlimit();
        boolean leftTop = climbSubsystem.getRightBottomlimit();


        while (climb != 0) {

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
