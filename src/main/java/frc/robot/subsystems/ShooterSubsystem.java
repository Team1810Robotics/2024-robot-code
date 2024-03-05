package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax top;
    private final CANSparkMax bottom;
    private final PIDController topPID;
    private final PIDController bottomPID;

    private double lastTopTime = Timer.getFPGATimestamp();
    private double lastBottomTime = Timer.getFPGATimestamp();
    private double lastTopPosition = 0;
    private double lastBottomPosition = 0;

    public ShooterSubsystem() {
        top = new CANSparkMax(TOP_MOTOR_ID, MotorType.kBrushless);
        bottom = new CANSparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        topPID = new PIDController(Top.kP, Top.kI, Top.kD);
        bottomPID = new PIDController(Bottom.kP, Bottom.kI, Bottom.kD);

        setSetpoint(HALF_SET_SPEED);

        // Shuffleboard.getTab("shooter").add("top shooter PID", topPID);
        // Shuffleboard.getTab("shooter").add("bottom shooter PID", bottomPID);
    }

    @Override
    public void periodic() {

        double topVelocity = getTopVelocity();
        top.set(-topPID.calculate(topVelocity, getTopSetpoint()));
        double bottomVelocity = getBottomVelocity();
        bottom.set(-bottomPID.calculate(bottomVelocity, getBottomSetpoint()));

        SmartDashboard.putNumber("vel top", topVelocity);
        SmartDashboard.putNumber("vel bot", bottomVelocity);
    }

    public void setSetpoint(double setpoint) {
        topPID.setSetpoint(setpoint);
        bottomPID.setSetpoint(setpoint);
    }

    public double getTopSetpoint() {
        return topPID.getSetpoint();
    }

    public double getBottomSetpoint() {
        return bottomPID.getSetpoint();
    }

    public void setSpeed(double motorSpeed) {
        top.set(motorSpeed);
        bottom.set(motorSpeed);
    }

    public void stop() {
        top.stopMotor();
        bottom.stopMotor();
    }

    public double getBottomVelocity() {
        double currentTime = Timer.getFPGATimestamp();
        double pos = bottom.getEncoder().getPosition();

        double deltaTime = currentTime - lastBottomTime;
        double deltaPosition = pos - lastBottomPosition;
        double velocity = deltaPosition / deltaTime;

        lastBottomTime = currentTime;
        lastBottomPosition = pos;

        return velocity;
    }

    public double getTopVelocity() {
        double currentTime = Timer.getFPGATimestamp();
        double pos = top.getEncoder().getPosition();

        double deltaTime = currentTime - lastTopTime;
        double deltaPosition = pos - lastTopPosition;
        double velocity = deltaPosition / deltaTime;

        lastTopTime = currentTime;
        lastTopPosition = pos;

        return velocity;
    }
}
