package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.OIConstants;

/** Class that "hides" the button bindings */
public final class IO {

    public static final Joystick joystick = new Joystick(OIConstants.DRIVER_JOYSTICK_PORT);
    public static final XboxController xboxController = new XboxController(OIConstants.MANIPULATOR_XBOX_PORT);

    // Joystick Buttons
    public static final JoystickButton joystick_trigger  = new JoystickButton(joystick, 1);
    public static final JoystickButton joystick_button2  = new JoystickButton(joystick, 2);
    public static final JoystickButton joystick_button3  = new JoystickButton(joystick, 3);
    public static final JoystickButton joystick_button4  = new JoystickButton(joystick, 4);
    public static final JoystickButton joystick_button5  = new JoystickButton(joystick, 5);
    public static final JoystickButton joystick_button6  = new JoystickButton(joystick, 6);
    public static final JoystickButton joystick_button7  = new JoystickButton(joystick, 7);
    public static final JoystickButton joystick_button8  = new JoystickButton(joystick, 8);
    public static final JoystickButton joystick_button9  = new JoystickButton(joystick, 9);
    public static final JoystickButton joystick_button10 = new JoystickButton(joystick, 10);
    public static final JoystickButton joystick_button11 = new JoystickButton(joystick, 11);

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