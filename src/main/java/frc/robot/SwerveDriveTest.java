package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

/**
 * Class to perform tests on the swerve drive.
 */
public class SwerveDriveTest
{

  /**
   * Set the angle of the modules to a given {@link Rotation2d}
   *
   * @param swerveDrive {@link SwerveDrive} to use.
   * @param moduleAngle {@link Rotation2d} to set every module to.
   */
  public static void angleModules(SwerveDrive swerveDrive, Rotation2d moduleAngle)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.setDesiredState(new SwerveModuleState(0, moduleAngle), false, true);
    }
  }

  /**
   * Power the drive motors for the swerve drive to a set duty cycle percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  Duty cycle percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotorsDutyCycle(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().set(percentage);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  DutyCycle percentage to send to angle motors.
   */
  public static void powerAngleMotors(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().set(percentage);
    }
  }

  /**
   * Set the modules to center to 0.
   *
   * @param swerveDrive Swerve Drive to control.
   */
  public static void centerModules(SwerveDrive swerveDrive)
  {
    angleModules(swerveDrive, Rotation2d.fromDegrees(0));
  }
}