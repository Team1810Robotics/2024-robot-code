package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder; 

public class ArmSubsystem extends TrapezoidProfileSubsystem {

CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
    
CANSparkMax motor2 = new CANSparkMax (0, MotorType.kBrushless);

CANcoder can = new CANcoder(0);

    public ArmSubsystem() {
        super(ArmConstants.CONSTRAINTS, ArmConstants.INITIAL_POSITION);

        
 
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

    @Override
    public void useState(State state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useState'");
    }
    
}
