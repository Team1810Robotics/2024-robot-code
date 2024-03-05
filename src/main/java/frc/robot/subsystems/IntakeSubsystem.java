package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final DigitalInput beam;

    public IntakeSubsystem() {
        motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
        beam = new DigitalInput(IntakeConstants.BEAM_BREAK_PORT);

        Shuffleboard.getTab("intake").addBoolean("hasNote", this::getBeamBreak);
    }

    public void setSpeed(double motorSpeed) {
        motor.set(motorSpeed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean getBeamBreak() {
        return beam.get();
    }
}
