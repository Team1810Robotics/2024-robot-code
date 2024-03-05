package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private VictorSPX leftMotor = new VictorSPX(Constants.ClimbConstants.LEFT_MOTOR_ID);
    private VictorSPX rightMotor = new VictorSPX(Constants.ClimbConstants.RIGHT_MOTOR_ID);

    DigitalInput leftBottom = new DigitalInput(Constants.ClimbConstants.LEFT_BOTTOM_LS);
    DigitalInput rightBottom = new DigitalInput(Constants.ClimbConstants.RIGHT_BOTTOM_LS);

    DigitalInput leftTop = new DigitalInput(Constants.ClimbConstants.LEFT_TOP_LS);
    DigitalInput rightTop = new DigitalInput(Constants.ClimbConstants.RIGHT_TOP_LS);

    public void setSpeed(double motorSpeed) {
        leftMotor.set(ControlMode.PercentOutput, motorSpeed);
        rightMotor.set(ControlMode.PercentOutput, motorSpeed);
    }

    public boolean getLeftBottomlimit() {
        return leftBottom.get();
    }

    public boolean getRightBottomlimit() {
        return rightBottom.get();
    }

    public boolean getRightToplimit() {
        return rightTop.get();
    }

    public boolean getLeftToplimit() {
        return leftTop.get();
    }
}
