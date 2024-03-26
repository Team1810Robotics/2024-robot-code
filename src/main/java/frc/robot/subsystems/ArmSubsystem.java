package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import lib.ArmFeedforward;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
    private CANSparkMax motorA = new CANSparkMax(MOTOR_A_ID, MotorType.kBrushless);
    private CANSparkMax motorB = new CANSparkMax(MOTOR_B_ID, MotorType.kBrushless);

    private CANcoder canCoder = new CANcoder(CANCODER_ID);

    private final PIDController controller;
    private final ArmFeedforward feedforward;

    public ArmSubsystem() {
        super(CONSTRAINTS);
        controller = new PIDController(kP, kI, kD);
        feedforward = new ArmFeedforward(ks, kg, kv);

        // controller.setTolerance(ARM_TOLERANCE);
        motorA.setInverted(true);
        motorB.setInverted(true);

        controller.setTolerance(1.0, 0.1);
        setpoint(INTAKE_POSITION);
        enable();

        Shuffleboard.getTab("arm").addNumber("canCoder pos", this::getMeasurement);
        Shuffleboard.getTab("arm").add("pid", controller);
        Shuffleboard.getTab("arm").addNumber("error", controller::getPositionError);
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        double pid = controller.calculate(getMeasurement(), setpoint.position);
        double feed = feedforward.calculate(getMeasurement(), getVelocity());

        motorA.setVoltage(pid + feed);
        motorB.setVoltage(pid + feed);
    }

    public double getMeasurement() {
        double position = canCoder.getAbsolutePosition().getValueAsDouble();
        double rads = position * TICKS_TO_RAD_CONVERSION;
        return rads - CANCODER_OFFSET;
    }

    /**
     * @param setpoint setpoint in degrees; 90 is when the arm is straight up and down
     */
    public void setpoint(double setpoint) {
        setpoint = Math.toRadians(setpoint - SETPOINT_OFFSET);
        super.setGoal(setpoint);
    }

    public double getVelocity() {
        return canCoder.getVelocity().getValueAsDouble() * TICKS_TO_RAD_CONVERSION;
    }

    public void setSpeed(double motorSpeed) {
        motorSpeed = MathUtil.clamp(motorSpeed, -1, 1);

        motorA.set(motorSpeed);
        motorB.set(motorSpeed);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public Command setpointCommand(double setpoint) {
        return Commands.run(() -> setpoint(setpoint), this);
    }
}
