package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

    private final ArmSubsystem armButton = new ArmSubsystem();
    private final ShooterSubsystem shooterButton = new ShooterSubsystem();
    private final IntakeSubsystem intakeButton = new IntakeSubsystem();

    private final XboxController manipulator = new XboxController(0);
    private final JoystickButton Button_A = new JoystickButton(manipulator, 1);
    private final JoystickButton Button_B = new JoystickButton(manipulator, 2);
    private final JoystickButton Button_X = new JoystickButton(manipulator, 3);
    //private final JoystickButton Button_Y = new JoystickButton(manipulator, 4);
    private final JoystickButton Button_LB = new JoystickButton(manipulator, 5);
    private final JoystickButton Button_RB = new JoystickButton(manipulator, 6);

    public RobotContainer() {

        configureBindings();
    }

    private void configureBindings() {

        Button_A.whileTrue(new ManualCommand(armButton, .1 ));
        Button_B.whileTrue(new ManualCommand(armButton, -.1 ));
        Button_X.whileTrue(new ShooterCommand(shooterButton, .75));
        Button_LB.whileTrue(new IntakeCommand(intakeButton, .75));
        Button_RB.whileTrue(new IntakeCommand(intakeButton, -.75));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
