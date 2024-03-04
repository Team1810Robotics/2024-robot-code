package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.OmniDrive;
import frc.robot.commands.swervedrive.TeleopDrive;
import frc.robot.commands.swervedrive.TeleopDriveSpeed;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer{

  public final static SwerveSubsystem drivebase = new SwerveSubsystem(Swerve.directory); // All reference to the swervedrive should use this instance. AKA: RobotContainer.drivebase.action... instead of making a new instance 
  
  public static CommandJoystick driver = new CommandJoystick(0);
  public static CommandJoystick rotationController = new CommandJoystick(1);

  CommandJoystick driverController = new CommandJoystick(1);
  XboxController joshBox = new XboxController(3);

  VisionSubsystem visionSubsystem = new VisionSubsystem(); //Commenting out did not fix overrun

  SendableChooser<Command> autoChooser;
  public SendableChooser<Command> driveChooser = new SendableChooser<>();
  
  public static GenericEntry dualDrive;

  Command visionDrive;
  Command teleopDrive;
  Command teleopDrive_twoJoy;
  Command speedDriveTest;
  Command testDrive;

  public RobotContainer(){

    // Configure the trigger bindings
    configureBindings();

    ShuffleboardTab Teleop = Shuffleboard.getTab("Teleoperated");
    dualDrive = Teleop.add("Dual Joystick Mode", true).getEntry();

    /**Speed Control - Drive on one joystick */
    speedDriveTest = new TeleopDriveSpeed(
       drivebase,
       () -> driver.getRawAxis(IOConstants.driveSpeedModAxis),
       () -> rotationController.getRawAxis(IOConstants.angleSpeedModAxis),
       () -> MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
       () -> MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
       () -> MathUtil.applyDeadband(driver.getRawAxis(IOConstants.driveOmegaAxis), IOConstants.DEADBAND)
    );

    /**Drive with 2 joysticks, automatically rotating toward a target AprilTag */
    visionDrive = new TeleopDrive(
      drivebase,
      () -> MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> -drivebase.visionTargetPIDCalc(visionSubsystem, -MathUtil.applyDeadband(rotationController.getRawAxis(IOConstants.driveOmegaAxis), IOConstants.DEADBAND), driver.button(1).getAsBoolean())
    );
   
    /**Drive on one joystick */
    teleopDrive = new TeleopDrive(
       drivebase,
       () -> MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
       () -> MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
       () -> MathUtil.applyDeadband(driver.getRawAxis(IOConstants.driveOmegaAxis), IOConstants.DEADBAND)
    );

    /**Drive with two joysticks */
    teleopDrive_twoJoy = new TeleopDrive(
      drivebase,
      () -> MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> MathUtil.applyDeadband(rotationController.getX(), IOConstants.DEADBAND)
    );

    Command OmniDrive = new OmniDrive(
      () -> driver.getY(),
      () -> driver.getX(),
      () -> driver.getThrottle(),
      () -> rotationController.getThrottle(),
      () -> dualDrive.getBoolean(false),
      visionSubsystem
    );

    /**Test - Used to stop drive form moving */
    testDrive = new TeleopDrive(drivebase, () -> 0, () -> 0, () -> 0);

    drivebase.setDefaultCommand(OmniDrive);

    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
  }

  private void configureBindings(){
    // Sets button 9 on Drive Joystick to zero gyro
    driver.button(IOConstants.resetGyroButton).onTrue(new InstantCommand(drivebase::zeroGyro));

    // A button that will interrupt drive and make the robot look at AprilTag
    driver.button(4).whileTrue(drivebase.aimAtTarget());
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }
}
