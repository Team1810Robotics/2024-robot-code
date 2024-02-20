package frc.robot.subsystems;

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

    // CHANGE: removed redundant calls to limit switches
    public boolean getLeftBottomlimit() {
        return leftBottom.get();
    }

    // CHANGE: removed redundant calls to limit switches
     public boolean getRightBottomlimit() {
        return rightBottom.get();
    }

    // CHANGE: removed redundant calls to limit switches
    public boolean getRightToplimit() {
        return rightTop.get();
    }

    // CHANGE: removed redundant calls to limit switches
    public boolean getLeftToplimit() {
        return  leftTop.get();
    }
}
