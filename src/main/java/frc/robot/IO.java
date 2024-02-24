package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.IOConstants;

/** Class that "hides" the button bindings */
public final class IO {

    public static final Joystick leftJoystick = new Joystick(IOConstants.LEFT_JOYSTICK_PORT);
    public static final Joystick rightJoystick = new Joystick(IOConstants.RIGHT_JOYSTICK_PORT);
    public static final XboxController xboxController = new XboxController(IOConstants.MANIPULATOR_XBOX_PORT);

    // Joystick Buttons
    public static final JoystickButton leftJoystick_trigger   = new JoystickButton(leftJoystick, 1);
    public static final JoystickButton leftJoystick_button2   = new JoystickButton(leftJoystick, 2);
    public static final JoystickButton leftJoystick_button3   = new JoystickButton(leftJoystick, 3);
    public static final JoystickButton leftJoystick_button4   = new JoystickButton(leftJoystick, 4);
    public static final JoystickButton leftJoystick_button5   = new JoystickButton(leftJoystick, 5);
    public static final JoystickButton leftJoystick_button6   = new JoystickButton(leftJoystick, 6);
    public static final JoystickButton leftJoystick_button7   = new JoystickButton(leftJoystick, 7);
    public static final JoystickButton leftJoystick_button8   = new JoystickButton(leftJoystick, 8);
    public static final JoystickButton leftJoystick_button9   = new JoystickButton(leftJoystick, 9);
    public static final JoystickButton leftJoystick_button10  = new JoystickButton(leftJoystick, 10);
    public static final JoystickButton leftJoystick_button11  = new JoystickButton(leftJoystick, 11);

    public static final JoystickButton rightJoystick_trigger  = new JoystickButton(leftJoystick, 1);
    public static final JoystickButton rightJoystick_button2  = new JoystickButton(leftJoystick, 2);
    public static final JoystickButton rightJoystick_button3  = new JoystickButton(leftJoystick, 3);
    public static final JoystickButton rightJoystick_button4  = new JoystickButton(leftJoystick, 4);
    public static final JoystickButton rightJoystick_button5  = new JoystickButton(leftJoystick, 5);
    public static final JoystickButton rightJoystick_button6  = new JoystickButton(leftJoystick, 6);
    public static final JoystickButton rightJoystick_button7  = new JoystickButton(leftJoystick, 7);
    public static final JoystickButton rightJoystick_button8  = new JoystickButton(leftJoystick, 8);
    public static final JoystickButton rightJoystick_button9  = new JoystickButton(leftJoystick, 9);
    public static final JoystickButton rightJoystick_button10 = new JoystickButton(leftJoystick, 10);
    public static final JoystickButton rightJoystick_button11 = new JoystickButton(leftJoystick, 11);

    // Xbox buttons
    public static final JoystickButton manipulatorXbox_A      = new JoystickButton(xboxController, XboxController.Button.kA.value);
    public static final JoystickButton manipulatorXbox_B      = new JoystickButton(xboxController, XboxController.Button.kB.value);
    public static final JoystickButton manipulatorXbox_X      = new JoystickButton(xboxController, XboxController.Button.kX.value);
    public static final JoystickButton manipulatorXbox_Y      = new JoystickButton(xboxController, XboxController.Button.kY.value);
    public static final JoystickButton manipulatorXbox_LB     = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton manipulatorXbox_RB     = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    public static final JoystickButton manipulatorXbox_Back   = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    public static final JoystickButton manipulatorXbox_Start  = new JoystickButton(xboxController, XboxController.Button.kStart.value);
    public static final JoystickButton manipulatorXbox_LStick = new JoystickButton(xboxController, XboxController.Button.kLeftStick.value);
    public static final JoystickButton manipulatorXbox_RStick = new JoystickButton(xboxController, XboxController.Button.kRightStick.value);

    private IO() {/* what does sleep feel like */}
}