package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax top;
    private final CANSparkMax bottom;

    public ShooterSubsystem() {
        top = new CANSparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
        bottom = new CANSparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        top.setInverted(true);
        bottom.setInverted(true);
    }

    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        top.set(speed);
        bottom.set(speed);
    }

    public void setVoltage(Measure<Voltage> volts) {
        top.setVoltage(volts.in(Volts));
        bottom.setVoltage(volts.in(Volts));
    }

    public void stop() {
        top.stopMotor();
        bottom.stopMotor();
    }
}
