package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
     
    CANSparkMax motor1 = new CANSparkMax(0, MotorType.kBrushless);

    DigitalInput input = new DigitalInput(0);

    public void setSpeed(double motorSpeed){    
        motor1.set(motorSpeed);

    }
    public boolean getBeamBreak(){
        return input.get();
    }

    public IntakeSubsystem(){
        
    }
}
