package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.TeleopDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer{

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  
  public static CommandJoystick driver = new CommandJoystick(0);
  CommandJoystick rotationController = new CommandJoystick(1);

  CommandJoystick driverController = new CommandJoystick(1); //Same?
  XboxController driverXbox = new XboxController(1);

  VisionSubsystem visionSubsystem = new VisionSubsystem();

  SendableChooser<Command> autoChooser;

  Command visionDrive;
  Command teleopDrive;
  Command teleopDrive_twoJoy;

  public RobotContainer(){
    //driveChooser.setDefaultOption(null, null);

    // Configure the trigger bindings
    configureBindings();

    /**Drive with 2 joysticks, automatically rotating toward a target AprilTag */
    visionDrive = new TeleopDrive(
      drivebase,
      () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> drivebase.rotAAA(-MathUtil.applyDeadband(rotationController.getRawAxis(IOConstants.driveOmegaAxis), 0.5)),
      () -> !driver.button(IOConstants.driveModeButton).getAsBoolean()  
    );
   
    /**Drive on one joystick */
    teleopDrive = new TeleopDrive(
       drivebase,
       () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
       () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
       () -> -MathUtil.applyDeadband(driver.getRawAxis(IOConstants.driveOmegaAxis), 0.5),
       () -> !driver.button(IOConstants.driveModeButton).getAsBoolean()  
    );

    /**Drive with two joysticks */
    teleopDrive_twoJoy = new TeleopDrive(
      drivebase,
      () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> -MathUtil.applyDeadband(rotationController.getRawAxis(IOConstants.driveOmegaAxis), 0.5),
      () -> !driver.button(IOConstants.driveModeButton).getAsBoolean()  
    );

    /* Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getY
        (), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2)); */

    drivebase.setDefaultCommand(teleopDrive_twoJoy);

    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  private void configureBindings(){

    driver.button(IOConstants.resetGyroButton).onTrue(new InstantCommand(drivebase::zeroGyro));

    IO.manipulatorXbox_Start.onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    IO.manipulatorXbox_Y.whileTrue(drivebase.aimAtTarget(visionSubsystem.getCamera()));

    IO.leftJoystick_trigger.whileTrue(visionDrive);
  }

  public Command getAutonomousCommand(){

    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake){
    
    drivebase.setMotorBrake(brake);
  }
}
