package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

    private final VictorSPX extender;

    public ExtenderSubsystem() {
        extender = new VictorSPX(ExtenderConstants.MOTOR_ID);
        extender.setNeutralMode(NeutralMode.Brake);
    }

    public void setSpeed(double speed) {
        extender.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        extender.set(ControlMode.PercentOutput, 0);
    }
}
