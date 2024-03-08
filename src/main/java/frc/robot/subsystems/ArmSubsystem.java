package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmSubsystem extends PIDSubsystem {
    private CANSparkMax armMotor = new CANSparkMax(MOTOR_A_ID, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(MOTOR_B_ID, MotorType.kBrushless);

    private CANcoder canCoder = new CANcoder(CANCODER_ID);

    private final ArmFeedforward feedforward;

    public ArmSubsystem() {
        super(new PIDController(kP, kI, kD));
        feedforward = new ArmFeedforward(ks, kg, kv);

        getController().setTolerance(ARM_TOLERANCE);
        motor2.follow(armMotor);

        setSetpoint(INITIAL_POSITION);

        Shuffleboard.getTab("arm").addNumber("canCoder pos", this::getMeasurement);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        double canCoderVelocity =
                canCoder.getVelocity().getValueAsDouble() * TICKS_TO_DEG_CONVERSION;
        double feedforwardOutput = feedforward.calculate(setpoint, canCoderVelocity);
        armMotor.set(output + feedforwardOutput);
    }

    @Override
    public double getMeasurement() {
        double position = canCoder.getPosition().getValueAsDouble();
        double degrees = position * TICKS_TO_DEG_CONVERSION;
        return degrees - CANCODER_OFFSET;
    }

    public void setSpeed(double motorSpeed) {
        armMotor.set(motorSpeed);
    }

    public void stop() {
        armMotor.stopMotor();
    }
}
