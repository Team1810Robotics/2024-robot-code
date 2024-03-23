package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax top;
    private final CANSparkMax bottom;
    //I hate robotics

    public ShooterSubsystem() {
        top = new CANSparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
        bottom = new CANSparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        top.setInverted(true);
        bottom.setInverted(true);
    }

    public void setSpeed(double volts) {
        top.setVoltage(volts);
        bottom.setVoltage(volts);
    }

    public void stop() {
        top.stopMotor();
        bottom.stopMotor();
    }
}
