package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
     
    CANSparkMax motor = new CANSparkMax(Constants.IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

    DigitalInput beam = new DigitalInput(Constants.IntakeConstants.BEAM_BREAK_PORT);

    public void setSpeed(double motorSpeed){    
        motor.set(motorSpeed);

    }
    public boolean getBeamBreak(){
        return beam.get();
    }

    public IntakeSubsystem(){
        
    }
}
