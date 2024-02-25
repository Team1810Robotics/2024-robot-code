package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TrapSubsystem extends SubsystemBase {

    /**
     * FIXME: should be private, should be a {@link com.ctre.phoenix.motorcontrol.can.VictorSPX} not
     * a {@link edu.wpi.first.wpilibj.motorcontrol.VictorSP}
     */
    VictorSP victor = new VictorSP(Constants.TrapConstants.VICTOR_MOTOR_PORT);

    // FIXME: rename to setSpeed() to keep consistent with other subsystems
    public void motorOn(double victorSpeed) {
        victor.set(victorSpeed);
    }

    // FIXME: remane to stop() to keep consistent with other subsystems
    public void motorOff() {
        victor.set(0);
    }
}
