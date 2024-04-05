package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final VictorSPX leftMotor;
    private final VictorSPX rightMotor;

    private final DigitalInput leftBottom;
    private final DigitalInput rightBottom;

    private final DigitalInput leftTop;
    private final DigitalInput rightTop;

    public ClimbSubsystem() {
        leftMotor = new VictorSPX(Constants.ClimbConstants.LEFT_MOTOR_ID);
        rightMotor = new VictorSPX(Constants.ClimbConstants.RIGHT_MOTOR_ID);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftBottom = new DigitalInput(Constants.ClimbConstants.LEFT_BOTTOM_LS);
        rightBottom = new DigitalInput(Constants.ClimbConstants.RIGHT_BOTTOM_LS);

        leftTop = new DigitalInput(Constants.ClimbConstants.LEFT_TOP_LS);
        rightTop = new DigitalInput(Constants.ClimbConstants.RIGHT_TOP_LS);

        Shuffleboard.getTab("climb").addBoolean("left bottom", () -> leftBottom.get());
        Shuffleboard.getTab("climb").addBoolean("right bottom", () -> rightBottom.get());
        Shuffleboard.getTab("climb").addBoolean("left top", () -> leftTop.get());
        Shuffleboard.getTab("climb").addBoolean("right top", () -> rightTop.get());
    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void climbUp() {
        if (!rightTop.get()) {
            setRightSpeed(ClimbConstants.CLIMB_SPEED);
        } else {
            setRightSpeed(0);
        }

        if (!leftTop.get()) {
            setLeftSpeed(ClimbConstants.CLIMB_SPEED);
        } else {
            setLeftSpeed(0);
        }
    }

    public void climbDown() {
        if (!rightBottom.get()) {
            setRightSpeed(-ClimbConstants.CLIMB_SPEED);
        } else {
            setRightSpeed(0);
        }

        if (!leftBottom.get()) {
            setLeftSpeed(-ClimbConstants.CLIMB_SPEED);
        } else {
            setLeftSpeed(0);
        }
    }

    public void stop() {
        setRightSpeed(0);
        setLeftSpeed(0);
    }

    public enum ClimbDirection {
        climbUp,
        climbOff,
        climbDown;
    }
}
