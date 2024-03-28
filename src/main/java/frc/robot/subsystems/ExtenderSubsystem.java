package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {

    private final VictorSPX extender;

    public ExtenderSubsystem() {
        extender = new VictorSPX(22);
        extender.setNeutralMode(NeutralMode.Brake);
    }

    public void forward() {
        extender.set(ControlMode.PercentOutput, -1.0);
    }

    public void reverse() {
        extender.set(ControlMode.PercentOutput, 1.0);
    }

    public void off() {
        extender.set(ControlMode.PercentOutput, 0.0);
    }
}
