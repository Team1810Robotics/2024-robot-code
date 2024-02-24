package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax bottomMotor = new CANSparkMax(Constants.ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    CANSparkMax topMotor = new CANSparkMax(Constants.ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);

    public ShooterSubsystem() {
        topMotor.setInverted(true);
    }

    public void setBottomSpeed(double bottomSpeed) {
        bottomMotor.set(bottomSpeed);
    }

    public void setTopSpeed(double topSpeed) {
        topMotor.set(topSpeed);
    }

    // FIXME: doesn't set both speeds?
    public void setBothSpeed(double motorSpeed) {
        topMotor.set(motorSpeed);
    }
}