package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {

    /*
     * CHANGE:
     * changed name of armMotor to better reflect the purpose of the variable
     * added motor & canCoder ids from constants
     * added pid controller constants
     */
    private CANSparkMax armMotor = new CANSparkMax(MOTOR1_PORT, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax (MOTOR2_PORT, MotorType.kBrushless);

    private CANcoder canCoder = new CANcoder(CANCODER_PORT);

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

    public void stop(){
        armMotor.stopMotor();
    }

    public void setGoal(double setpoint){
        armMotor.set(pid.calculate(getposition(), setpoint));
    }

}