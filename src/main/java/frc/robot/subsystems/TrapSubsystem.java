package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapSubsystem extends SubsystemBase {
    
    VictorSP victor = new VictorSP(Constants.TrapConstants.VICTOR_MOTOR_PORT);

    public void motorOn(double victorSpeed){    
        victor.set(victorSpeed);

    }
    
    public void motorOff(){
        victor.set(0);
        
    }
}

