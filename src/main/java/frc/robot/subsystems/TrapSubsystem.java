package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapSubsystem extends SubsystemBase {

    VictorSPX victor = new VictorSPX(Constants.TrapConstants.VICTOR_MOTOR_ID);


    public void setSpeed(double victorSpeed) {
        victor.set(ControlMode.PercentOutput, victorSpeed);
    }

    public void stop() {
        victor.set(ControlMode.PercentOutput, 0);
    }
}
