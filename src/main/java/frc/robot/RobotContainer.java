package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.controller.IO.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.*;
import frc.robot.commands.Auto.Align;
import frc.robot.commands.Auto.Position;
import frc.robot.subsystems.*;

public class RobotContainer {

    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    public final DriveSubsystem driveSubsystem = new DriveSubsystem(SwerveConstants.DIRECTORY);
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    public final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();

    public final Child children = new Child();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        @SuppressWarnings("unused")
        Command visDrive =
                new TeleopDriveVis(
                        driveSubsystem,
                        visionSubsystem,
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(-driver.getZ(), IOConstants.DEADBAND),
                        () -> driver.getTrigger());

        @SuppressWarnings("unused")
        Command visDrive_two =
                new TeleopDriveVis(
                        driveSubsystem,
                        visionSubsystem,
                        () -> MathUtil.applyDeadband(driver.getY(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(driver.getX(), IOConstants.DEADBAND),
                        () -> MathUtil.applyDeadband(-rotation.getX(), IOConstants.DEADBAND),
                        () -> driver.getTrigger());

        driveSubsystem.setDefaultCommand(visDrive_two);

        NamedCommands.registerCommand(
                "Shoot",
                new AimShoot(
                        shooterSubsystem, intakeSubsystem, armSubsystem, visionSubsystem, false));
        NamedCommands.registerCommand(
                "IntakePosition", new Position(armSubsystem, ArmConstants.INTAKE_POSITION));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intakeSubsystem, 0.75));
        NamedCommands.registerCommand("Align", new Align(driveSubsystem, visionSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        driver_button4.onTrue(Commands.runOnce(driveSubsystem::zeroGyro));

        driver_trigger
                .whileTrue(
                        new AimShoot(
                                shooterSubsystem,
                                intakeSubsystem,
                                armSubsystem,
                                visionSubsystem,
                                true))
                .onFalse(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION));

        driver_button2
                .whileTrue(
                        new FeederShot(
                                shooterSubsystem,
                                intakeSubsystem,
                                armSubsystem,
                                visionSubsystem,
                                true))
                .onFalse(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION));

        driver_button12.whileTrue(new Align(driveSubsystem, visionSubsystem));

        box_intake.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        box_outtake.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
        box_intakePos
                .onTrue(armSubsystem.setpointCommand(ArmConstants.INTAKE_POSITION))
                .onTrue(children.add());
        box_travelPos
                .onTrue(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION))
                .onTrue(children.doubleChildren());
        box_up.onTrue(armSubsystem.setpointCommand(ArmConstants.CLOSE_SHOOT_POSITION))
                .onTrue(children.subtract());
        box_climb
                .onTrue(new PrintCommand("Gave Children Lasagna"))
                .onFalse(new PrintCommand("Children are starving!"));

        xbox_RStick.whileTrue(
                new ShooterCommand(shooterSubsystem, intakeSubsystem, false, () -> false));
        xbox_A.onTrue(armSubsystem.setpointCommand(ArmConstants.INTAKE_POSITION));
        xbox_Y.onTrue(armSubsystem.setpointCommand(ArmConstants.DRIVE_POSITION));
        xbox_B.whileTrue(new IntakeCommand(intakeSubsystem, 0.75));
        xbox_X.whileTrue(new IntakeCommand(intakeSubsystem, -1.0));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        driveSubsystem.setMotorBrake(brake);
    }
}
