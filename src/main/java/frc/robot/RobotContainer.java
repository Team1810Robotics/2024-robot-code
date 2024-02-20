package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.IO.*;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    // CHANGE: move the controller bindings to a separate class (see IO.java)

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        manipulatorXbox_A.onTrue(new ManualCommand(armSubsystem, .1)).onFalse(new ManualCommand(armSubsystem, 0));
        manipulatorXbox_B.onTrue(new ManualCommand(armSubsystem, -.1)).onFalse(new ManualCommand(armSubsystem, 0));
        manipulatorXbox_Y.onTrue(new ClimbCommand(climbSubsystem, .5));
        manipulatorXbox_X.onTrue(new ClimbCommand(climbSubsystem, -.5));

        manipulatorXbox_LB.whileTrue(new IntakeCommand(intakeSubsystem, .75));
        manipulatorXbox_RB.whileTrue(new IntakeCommand(intakeSubsystem, -.75));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
