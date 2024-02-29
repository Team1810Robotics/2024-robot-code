package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax bottomMotor =
            new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    CANSparkMax topMotor = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);

    public ShooterSubsystem() {
        topMotor.setInverted(true);
    }

    public void setBottomSpeed(double bottomSpeed) {
        bottomMotor.set(bottomSpeed);
    }

    public void setTopSpeed(double topSpeed) {
        topMotor.set(topSpeed);
    }

    public void setSpeed(double motorSpeed) {
        topMotor.set(motorSpeed);
    }

     public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
    }
}
