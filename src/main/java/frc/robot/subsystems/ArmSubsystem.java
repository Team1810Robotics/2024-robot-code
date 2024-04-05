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
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem {
    private CANSparkMax motorA;
    private CANSparkMax motorB;

    private CANcoder canCoder = new CANcoder(CANCODER_ID);

    private final PIDController controller;

    public ArmSubsystem() {
        super(CONSTRAINTS);
        controller = new PIDController(kP, kI, kD);

        motorA = new CANSparkMax(MOTOR_A_ID, MotorType.kBrushless);
        motorB = new CANSparkMax(MOTOR_B_ID, MotorType.kBrushless);

        motorA.setInverted(true);
        motorB.setInverted(true);

        controller.setIZone(kIZ);
        controller.setTolerance(Math.toRadians(2.0));
        setpoint(INTAKE_POSITION);
        enable();

        setupShuffleboard();
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        double pid = controller.calculate(getMeasurement(), setpoint.position);

        motorA.setVoltage(pid);
        motorB.setVoltage(pid);
    }

    public double getMeasurement() {
        double position = canCoder.getAbsolutePosition().getValueAsDouble();
        double rads = position * TICKS_TO_RAD_CONVERSION;
        return rads - CANCODER_OFFSET;
    }

    public double getVelocity() {
        double velocity = canCoder.getVelocity().getValueAsDouble();
        return velocity * TICKS_TO_RAD_CONVERSION;
    }

    public double getMeasurementDegrees() {
        return Math.toDegrees(getMeasurement());
    }

    /**
     * @param setpoint setpoint in degrees; 90° is when the arm is straight up and down
     */
    public void setpoint(double setpoint) {
        double clampSetpoint = MathUtil.clamp(setpoint, 41, 95.0);
        clampSetpoint = Math.toRadians(clampSetpoint);
        super.setGoal(clampSetpoint);
    }

    public void stop() {
        motorA.stopMotor();
        motorB.stopMotor();
    }

    public boolean atSetpoint() {
        double intakeRadians = Math.toRadians(ArmConstants.INTAKE_POSITION);
        if (controller.getSetpoint() == intakeRadians) return false;
        return controller.atSetpoint();
    }

    public boolean atSetpointRaw() {
        return controller.atSetpoint();
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public double getSetpointDegrees() {
        return Math.toDegrees(getSetpoint());
    }

    public Command setpointCommand(double setpoint) {
        return Commands.runOnce(() -> setpoint(setpoint));
    }

    private void setupShuffleboard() {
        Shuffleboard.getTab("arm").add("pid", controller);
        Shuffleboard.getTab("arm").addNumber("canCoder pos", this::getMeasurement);
        Shuffleboard.getTab("arm").addNumber("canCoder pos degrees", this::getMeasurementDegrees);
        Shuffleboard.getTab("arm").addNumber("error", controller::getPositionError);
        Shuffleboard.getTab("arm").addBoolean("atSetpoint", this::atSetpoint);
        Shuffleboard.getTab("arm").addBoolean("atSetpointRaw", this::atSetpointRaw);
        Shuffleboard.getTab("arm").addNumber("arm setpoint", this::getSetpointDegrees);
    }
}
