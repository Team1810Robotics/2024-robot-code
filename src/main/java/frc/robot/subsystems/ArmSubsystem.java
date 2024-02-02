package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder; 

public class ArmSubsystem extends SubsystemBase {

CANSparkMax motor = new CANSparkMax(Constants.ArmConstants.MOTOR1_PORT, MotorType.kBrushless);
    
CANSparkMax motor2 = new CANSparkMax (Constants.ArmConstants.MOTOR2_PORT, MotorType.kBrushless);

CANcoder can = new CANcoder(0);

ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

PIDController pid = new PIDController(0, 0, 0);

    public ArmSubsystem() {
     
     motor2.follow(motor);
    }

   public void setSpeed(double motorSpeed){    
        motor.set(motorSpeed);

}

   public double getposition(){
        can.getPosition();

        return can.getPosition().getValue();
}

   public void stop(){
        motor.stopMotor();
}

   public void setGoal(double setpoint){
        motor.set(pid.calculate(getposition(), setpoint));

  }

 
    }
    

