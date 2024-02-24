package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor = new CANSparkMax(MOTOR1_ID, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(MOTOR2_ID, MotorType.kBrushless);

    private CANcoder canCoder = new CANcoder(CANCODER_ID);

    private PIDController pid = new PIDController(kP, kI, kD);

    public ArmSubsystem() {
        motor2.follow(armMotor);
    }

    public void setSpeed(double motorSpeed) {
        armMotor.set(motorSpeed);
    }

    public double getposition() {
        return canCoder.getPosition().getValue();
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public void setGoal(double setpoint) {
        armMotor.set(pid.calculate(getposition(), setpoint));
    }
}
