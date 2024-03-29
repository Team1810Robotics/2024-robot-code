package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LEDSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private Timer disabledTimer;

    private GenericEntry setpointEntry;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable.  This will let the
        // robot stop
        // immediately when disabled, but then also let it be pushed more
        disabledTimer = new Timer();

        double setpoint = m_robotContainer.armSubsystem.getMeasurementDegrees();
        m_robotContainer.armSubsystem.setpoint(setpoint);

        setpointEntry = Shuffleboard.getTab("arm").add("PID setpoint", setpoint).getEntry();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update the LEDSubsystem
        boolean hasNote = m_robotContainer.intakeSubsystem.hasNote();
        boolean hasTarget = m_robotContainer.visionSubsystem.hasSpeakerTarget();
        boolean isAligned = m_robotContainer.visionSubsystem.isAligned();

        if (!hasNote) {
            LEDSubsystem.setState(LEDSubsystem.LEDState.off);
        } else if (isAligned) {
            LEDSubsystem.setState(LEDSubsystem.LEDState.isAligned);
        } else if (hasTarget) {
            LEDSubsystem.setState(LEDSubsystem.LEDState.hasTarget);
        } else if (!hasTarget) {
            LEDSubsystem.setState(LEDSubsystem.LEDState.noTarget);
        } else {
            // should never happen ¯\_(ツ)_/¯
            LEDSubsystem.setState(LEDSubsystem.LEDState.off);
        }

        // m_robotContainer.armSubsystem.setpoint(setpointEntry.getDouble(62));
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(SwerveConstants.WHEEL_LOCK_TIME)) {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.setMotorBrake(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
}
