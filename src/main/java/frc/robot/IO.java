package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;

/** Class that "hides" the button bindings */
public final class IO {

    public static final Joystick driver = new Joystick(IOConstants.DRIVE_JOYSTICK_PORT);
    public static final Joystick rotation = new Joystick(IOConstants.ROTATION_JOYSTICK_PORT);
    public static final XboxController xboxController = new XboxController(IOConstants.MANIPULATOR_XBOX_PORT);
    public static final Box box = new Box(4);

    // Joystick Buttons
    public static final JoystickButton driver_trigger = new JoystickButton(driver, 1);
    public static final JoystickButton driver_button2 = new JoystickButton(driver, 2);
    public static final JoystickButton driver_button3 = new JoystickButton(driver, 3);
    public static final JoystickButton driver_button4 = new JoystickButton(driver, 4);
    public static final JoystickButton driver_button5 = new JoystickButton(driver, 5);
    public static final JoystickButton driver_button6 = new JoystickButton(driver, 6);
    public static final JoystickButton driver_button7 = new JoystickButton(driver, 7);
    public static final JoystickButton driver_button8 = new JoystickButton(driver, 8);
    public static final JoystickButton driver_button9 = new JoystickButton(driver, 9);
    public static final JoystickButton driver_button10 = new JoystickButton(driver, 10);
    public static final JoystickButton driver_button11 = new JoystickButton(driver, 11);

    public static final JoystickButton rightJoystick_trigger =  new JoystickButton(rotation, 1);
    public static final JoystickButton rightJoystick_button2 =  new JoystickButton(rotation, 2);
    public static final JoystickButton rightJoystick_button3 =  new JoystickButton(rotation, 3);
    public static final JoystickButton rightJoystick_button4 =  new JoystickButton(rotation, 4);
    public static final JoystickButton rightJoystick_button5 =  new JoystickButton(rotation, 5);
    public static final JoystickButton rightJoystick_button6 =  new JoystickButton(rotation, 6);
    public static final JoystickButton rightJoystick_button7 =  new JoystickButton(rotation, 7);
    public static final JoystickButton rightJoystick_button8 =  new JoystickButton(rotation, 8);
    public static final JoystickButton rightJoystick_button9 =  new JoystickButton(rotation, 9);
    public static final JoystickButton rightJoystick_button10 = new JoystickButton(rotation, 10);
    public static final JoystickButton rightJoystick_button11 = new JoystickButton(rotation, 11);

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

    public static final JoystickButton box_climb          = new JoystickButton(box, Box.Button.climb.value);
    public static final JoystickButton box_low            = new JoystickButton(box, Box.Button.low.value);
    public static final JoystickButton box_autoTarget     = new JoystickButton(box, Box.Button.autoTarget.value);
    public static final JoystickButton box_trimUp         = new JoystickButton(box, Box.Button.trimUp.value);
    public static final JoystickButton box_trimDown       = new JoystickButton(box, Box.Button.trimDown.value);
    public static final JoystickButton box_in             = new JoystickButton(box, Box.Button.in.value);
    public static final JoystickButton box_out            = new JoystickButton(box, Box.Button.out.value);
    public static final JoystickButton box_shoot          = new JoystickButton(box, Box.Button.shoot.value);
    public static final JoystickButton box_unused         = new JoystickButton(box, Box.Button.unused.value);

    private IO() {
        /* what does sleep feel like */
    }
}