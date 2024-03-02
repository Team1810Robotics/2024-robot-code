package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax top = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private CANSparkMax bottom =
            new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final PIDController controller =
            new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    private double lastTime = Timer.getFPGATimestamp();
    private double lastTopPosition = 0;
    private double lastBottomPosition = 0;

    public ShooterSubsystem() {
        controller.setTolerance(ShooterConstants.TOLERANCE);
        top.setInverted(true);

        setSetpoint(ShooterConstants.HALF_SET_SPEED);
    }

    @Override
    public void periodic() {
        /**
         * rev velocity calculation is broken at the moment so we are using the encoder position to
         * calculate the velocity
         */
        double[] velocities = getVelocities();

        top.set(controller.calculate(velocities[0], getSetpoint()));
        bottom.set(controller.calculate(velocities[1], getSetpoint()));
    }

    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return controller.getSetpoint();
    }

    public void setSpeed(double motorSpeed) {
        top.set(motorSpeed);
        bottom.set(motorSpeed);
    }

    public void stop() {
        top.stopMotor();
        bottom.stopMotor();
    }

    public double[] getVelocities() {
        double currentTime = Timer.getFPGATimestamp();
        double topPosition = top.getEncoder().getPosition();
        double bottomPosition = bottom.getEncoder().getPosition();

        double deltaTime = currentTime - lastTime;
        double topVelocity = (topPosition - lastTopPosition) / deltaTime;
        double bottomVelocity = (bottomPosition - lastBottomPosition) / deltaTime;

        lastTime = currentTime;
        lastTopPosition = topPosition;
        lastBottomPosition = bottomPosition;

        return new double[] {topVelocity, bottomVelocity};
    }
}
