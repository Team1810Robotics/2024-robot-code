package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  CommandJoystick driverController = new CommandJoystick(1);
  XboxController joshBox = new XboxController(3);

  VisionSubsystem visionSubsystem = new VisionSubsystem();

  SendableChooser<Command> autoChooser;

  Command visionDrive;
  Command teleopDrive;
  Command teleopDrive_twoJoy;
  Command driveFieldOrientedAnglularVelocity;

  public RobotContainer(){

    // Configure the trigger bindings
    configureBindings();

    /**Drive with 2 joysticks, automatically rotating toward a target AprilTag */
    visionDrive = new TeleopDrive(
      drivebase,
      () -> -MathUtil.applyDeadband(driver.getY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),   
      () -> drivebase.visionTargetPIDCalc(-MathUtil.applyDeadband(rotationController.getRawAxis(IOConstants.driveOmegaAxis), 0.5)),
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

    /**YAGSL build in field oriented drive*/
    driveFieldOrientedAnglularVelocity = drivebase.driveCommand( //TODO Test? - Might be useful for visionDrive
      () -> MathUtil.applyDeadband(driver.getY
      (), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driver.getX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driver.getRawAxis(2));

    // Create a SendableChooser to select the drive command
    SendableChooser<Command> driveChooser = new SendableChooser<>();
    driveChooser.setDefaultOption("Vision Drive", visionDrive);
    driveChooser.addOption("Teleop Drive",teleopDrive);
    driveChooser.addOption("Teleop Drive (Two Joysticks)", teleopDrive_twoJoy);
    driveChooser.addOption("Drive Field Oriented (Needs Testing)", driveFieldOrientedAnglularVelocity);
    Shuffleboard.getTab("Teleoperated").add("Drive Command", driveChooser);

    //Shuffleboard.getTab("Teleoperated").add("Camera", visionSubsystem.getCamera());

    drivebase.setDefaultCommand(driveChooser.getSelected());

    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  private void configureBindings(){

    /* driver.button(IOConstants.resetGyroButton).onTrue(new InstantCommand(drivebase::zeroGyro));

    IO.manipulatorXbox_Start.onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    IO.manipulatorXbox_Y.whileTrue(drivebase.aimAtTarget());

    IO.leftJoystick_trigger.whileTrue(visionDrive); */
  }

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake){
    drivebase.setMotorBrake(brake);
  }
}
