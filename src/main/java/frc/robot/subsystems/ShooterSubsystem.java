package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    CANSparkMax motor1 = new CANSparkMax(0, MotorType.kBrushless);

    CANSparkMax motor2 = new CANSparkMax(0, MotorType.kBrushless);
      

    public void setSpeed1(double motor1Speed){    
        motor1.set(motor1Speed);
    }

    public void setSpeed2(double motor2Speed){    
        motor2.set(motor2Speed);
              
    }

    public void setBothSpeed(double motorSpeed){    
        motor1.set(motorSpeed);
        motor2.set(motorSpeed);
    }
    public ShooterSubsystem(){
        motor2.setInverted(true);
    }
}