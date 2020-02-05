package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utilities.ButtonCombination;

/**
 * This class holds the static methods of the buttons, in cases where commands want to read the controller values.
 * This should not be confused with RobotContainer, which sets up autonomous commands and maps commands to buttons.
 */
public class OI {
    public static final int xboxLeftXStick = 0;
    public static final int xboxLeftYStick = 1;
    public static final int xboxRightXStick = 4;
    public static final int xboxRightYStick = 5;
    //Controllers are defined here. (Only the RobotContainer and OI should access them)
    static Joystick rightStick = new Joystick(0);
    static Joystick leftStick = new Joystick(1);
    static XboxController xbox = new XboxController(2);
    public static ButtonCombination select_start = new ButtonCombination(xbox, 7, 8);
    public static JoystickButton lb = new JoystickButton(xbox, 6);
    public static JoystickButton rb = new JoystickButton(xbox, 5);
    public static JoystickButton start = new JoystickButton(xbox, 8);
    public static JoystickButton select = new JoystickButton(xbox, 7);
    public static JoystickButton x = new JoystickButton(xbox, 4);
    public static JoystickButton y = new JoystickButton(xbox, 3);
    public static JoystickButton b = new JoystickButton(xbox, 2);
    public static JoystickButton a = new JoystickButton(xbox, 1);

    public static double getLeftXboxX() {
        return xbox.getRawAxis(xboxLeftXStick);
    }

    public static double getLeftXboxY() {
        return xbox.getRawAxis(xboxLeftYStick);
    }

    public static double getRightXboxY() {
        return xbox.getRawAxis(xboxRightYStick);
    }

    public static double getRightXboxX() {
        return xbox.getRawAxis(xboxRightXStick);
    }

    public static double getLeftStickForward(){
        return -leftStick.getY();
    }

    public static double getRightStickForward(){
        return -rightStick.getY();
    }
}
