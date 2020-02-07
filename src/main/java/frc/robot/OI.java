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
    static final Joystick rightStick = new Joystick(0);
    static final Joystick leftStick = new Joystick(1);
    static final XboxController xbox = new XboxController(2);
    public final static ButtonCombination select_start = new ButtonCombination(xbox, 7, 8);
    public final static JoystickButton lb = new JoystickButton(xbox, 6);
    public final static JoystickButton rb = new JoystickButton(xbox, 5);
    public final static JoystickButton start = new JoystickButton(xbox, 8);
    public final static JoystickButton select = new JoystickButton(xbox, 7);
    public final static JoystickButton x = new JoystickButton(xbox, 3);
    public final static JoystickButton y = new JoystickButton(xbox, 4);
    public final static JoystickButton b = new JoystickButton(xbox, 2);
    public final static JoystickButton a = new JoystickButton(xbox, 1);

    public static double getXboxLX() {
        return xbox.getRawAxis(xboxLeftXStick);
    }

    public static double getXboxLY() {
        return xbox.getRawAxis(xboxLeftYStick);
    }

    public static double getXboxRY() {
        return xbox.getRawAxis(xboxRightYStick);
    }

    public static double getXboxRX() {
        return xbox.getRawAxis(xboxRightXStick);
    }

    public static double getLeftStickForward(){
        return -leftStick.getY();
    }

    public static double getRightStickForward(){
        return -rightStick.getY();
    }
}
