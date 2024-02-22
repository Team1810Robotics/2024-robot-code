package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IO;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer{

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  
  CommandJoystick driver = new CommandJoystick(0);
  CommandJoystick rotationController = new CommandJoystick(1);

  CommandJoystick driverController = new CommandJoystick(1); //Same?
  XboxController driverXbox = new XboxController(0); 

  SendableChooser<Command> autoChooser;

  public RobotContainer(){
    //driveChooser.setDefaultOption(null, null);

    // Configure the trigger bindings
    configureBindings();

    Command teleopDrive = new TeleopDrive(
      drivebase,
      () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> -MathUtil.applyDeadband(driver.getRawAxis(IO.driveOmegaAxis), 0.5),
      () -> !driver.button(IO.driveModeButton).getAsBoolean()  
    );
    Command teleopDrive_twoJoy = new TeleopDrive(
      drivebase,
      () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(rotationController.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> -MathUtil.applyDeadband(driver.getRawAxis(IO.driveOmegaAxis), 0.5),
      () -> !driver.button(IO.driveModeButton).getAsBoolean()  
    );

    /* Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRawAxis(2)); */

    drivebase.setDefaultCommand(teleopDrive);

    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  private void configureBindings(){

    driver.button(IO.resetGyroButton).onTrue(new InstantCommand(drivebase::zeroGyro));

    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
  }

  public Command getAutonomousCommand(){

    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake){
    
    drivebase.setMotorBrake(brake);
  }
}
