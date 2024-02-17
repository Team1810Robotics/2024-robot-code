package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    
    CANSparkMax leftMotor = new CANSparkMax(Constants.ClimbConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);

    CANSparkMax rightMotor = new CANSparkMax(Constants.ClimbConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

    DigitalInput leftBottom = new DigitalInput(Constants.ClimbConstants.LEFT_BOTTOM_LS);

    DigitalInput rightBottom = new DigitalInput(Constants.ClimbConstants.RIGHT_BOTTOM_LS);

    DigitalInput leftTop = new DigitalInput(Constants.ClimbConstants.LEFT_TOP_LS);

    DigitalInput rightTop = new DigitalInput(Constants.ClimbConstants.RIGHT_TOP_LS);

    
    public void setMotorSpeed(double motorSpeed){
    
        leftMotor.set(motorSpeed);
        rightMotor.set(motorSpeed);
}

    public boolean getLeftBottomlimit(){

        leftBottom.get();
        return leftBottom.get();
    }

     public boolean getRightBottomlimit(){

        rightBottom.get();
        return rightBottom.get();
    }

    public boolean getRightToplimit(){
        rightTop.get();
        return rightTop.get();
    }
    
    public boolean getLeftToplimit(){
        leftTop.get();
        return  leftTop.get();
    }
}
