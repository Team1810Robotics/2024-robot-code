package frc.robot.controller;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.controller.Box.Button.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;

/** Class that "hides" the button bindings */
public final class IO {

    public static final Joystick driver = new Joystick(IOConstants.LEFT_JOYSTICK_PORT);
    public static final Joystick rotation = new Joystick(IOConstants.RIGHT_JOYSTICK_PORT);
    public static final XboxController xboxController = new XboxController(IOConstants.XBOX_PORT);
    public static final Box box = new Box(IOConstants.BOX_PORT);

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
    public static final JoystickButton driver_button12 = new JoystickButton(driver, 12);

    public static final JoystickButton rotation_trigger = new JoystickButton(driver, 1);
    public static final JoystickButton rotation_button2 = new JoystickButton(driver, 2);
    public static final JoystickButton rotation_button3 = new JoystickButton(driver, 3);
    public static final JoystickButton rotation_button4 = new JoystickButton(driver, 4);
    public static final JoystickButton rotation_button5 = new JoystickButton(driver, 5);
    public static final JoystickButton rotation_button6 = new JoystickButton(driver, 6);
    public static final JoystickButton rotation_button7 = new JoystickButton(driver, 7);
    public static final JoystickButton rotation_button8 = new JoystickButton(driver, 8);
    public static final JoystickButton rotation_button9 = new JoystickButton(driver, 9);
    public static final JoystickButton rotation_button10 = new JoystickButton(driver, 10);
    public static final JoystickButton rotation_button11 = new JoystickButton(driver, 11);

    // Xbox buttons
    public static final JoystickButton manipulatorXbox_A =
            new JoystickButton(xboxController, kA.value);
    public static final JoystickButton manipulatorXbox_B =
            new JoystickButton(xboxController, kB.value);
    public static final JoystickButton manipulatorXbox_X =
            new JoystickButton(xboxController, kX.value);
    public static final JoystickButton manipulatorXbox_Y =
            new JoystickButton(xboxController, kY.value);
    public static final JoystickButton manipulatorXbox_LB =
            new JoystickButton(xboxController, kLeftBumper.value);
    public static final JoystickButton manipulatorXbox_RB =
            new JoystickButton(xboxController, kRightBumper.value);
    public static final JoystickButton manipulatorXbox_Back =
            new JoystickButton(xboxController, kBack.value);
    public static final JoystickButton manipulatorXbox_Start =
            new JoystickButton(xboxController, kStart.value);
    public static final JoystickButton manipulatorXbox_LStick =
            new JoystickButton(xboxController, kLeftStick.value);
    public static final JoystickButton manipulatorXbox_RStick =
            new JoystickButton(xboxController, kRightStick.value);

    public static final JoystickButton box_intake = new JoystickButton(box, in.get());
    public static final JoystickButton box_outtake = new JoystickButton(box, out.get());
    public static final JoystickButton box_climbUp = new JoystickButton(box, up.get());
    public static final JoystickButton box_climbDown = new JoystickButton(box, down.get());
    public static final JoystickButton box_climbPos = new JoystickButton(box, climbPos.get());
    public static final JoystickButton box_travelPos = new JoystickButton(box, travelPos.get());
    public static final JoystickButton box_intakePos = new JoystickButton(box, intakePos.get());

    private IO() {
        /* what does sleep feel like */
    }
}
