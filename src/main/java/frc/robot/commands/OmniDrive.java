package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveController;

public class OmniDrive extends Command {

  private final SwerveSubsystem driveSubsystem;
  private CommandJoystick driver;
  private CommandJoystick rotation;
  private Boolean driveMode = true;
  private BooleanSupplier dualJoystick;
  private DoubleSupplier yAxis;
  private DoubleSupplier xAxis;
  private DoubleSupplier driveSpeed;
  private DoubleSupplier rotationSpeed;

  private final SwerveController controller;
  private VisionSubsystem visionSubsystem;

  public OmniDrive(SwerveSubsystem driveSubsystem, DoubleSupplier yAxis, DoubleSupplier xAxis, DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed, BooleanSupplier dualJoystick, CommandJoystick rotation, CommandJoystick driver, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.yAxis = yAxis;
    this.xAxis = xAxis;
    this.driveSpeed = driveSpeed;
    this.rotationSpeed = rotationSpeed;
    this.dualJoystick = dualJoystick;
    this.rotation = rotation;
    this.driver = driver;
    this.visionSubsystem = visionSubsystem;
    this.controller = driveSubsystem.getSwerveController();
    
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double speedMult = (driveSpeed.getAsDouble() + 1) / 2;
    double rotationMult = (rotationSpeed.getAsDouble() + 1) / 2;
    double xVelocity = xAxis.getAsDouble() * speedMult;
    double yVelocity = yAxis.getAsDouble() * rotationMult;
    double angVelocity;

    if (dualJoystick.getAsBoolean() == true){
      angVelocity = driveSubsystem.visionTargetPIDCalc(
                                        visionSubsystem,
                                        MathUtil.applyDeadband(
                                                rotation.getY(), IOConstants.DEADBAND),
                                        driver.button(1).getAsBoolean());
    } else{
      angVelocity = driveSubsystem.visionTargetPIDCalc(
                                        visionSubsystem,
                                        MathUtil.applyDeadband(
                                                driver.getZ(), IOConstants.DEADBAND),
                                        driver.button(1).getAsBoolean());

    System.out.println(rotationMult + "    " + rotationMult + "     " + xVelocity + "     " + yVelocity + "    ");

    driveSubsystem.drive(
                new Translation2d(
                        xVelocity * Swerve.maxVelocity,
                        yVelocity * Swerve.maxVelocity),
                angVelocity * controller.config.maxAngularVelocity,
                driveMode);
    }
  }
}