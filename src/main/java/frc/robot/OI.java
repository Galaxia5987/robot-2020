package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.utilities.ButtonCombination;

/**
 * This class holds the static methods of the buttons, in cases where commands want to read the controller values.
 * This should not be confused with RobotContainer, which sets up autonomous commands and maps commands to buttons.
 */
public class OI {
    //Controllers are defined here. (Only the RobotContainer and OI should access them)
    static final Joystick rightStick = new Joystick(1);
    static final Joystick leftStick = new Joystick(0);
    static final XboxController xbox = new XboxController(2);

    public final static JoystickButton a = new JoystickButton(xbox, XboxController.Button.kA.value);
    public final static JoystickButton b = new JoystickButton(xbox, XboxController.Button.kB.value);
    public final static JoystickButton x = new JoystickButton(xbox, XboxController.Button.kX.value);
    public final static JoystickButton y = new JoystickButton(xbox, XboxController.Button.kY.value);
    public final static JoystickButton lb = new JoystickButton(xbox, XboxController.Button.kBumperLeft.value);
    public final static JoystickButton rb = new JoystickButton(xbox, XboxController.Button.kBumperRight.value);
    public final static JoystickButton back = new JoystickButton(xbox, XboxController.Button.kBack.value);
    public final static JoystickButton start = new JoystickButton(xbox, XboxController.Button.kStart.value);
    public final static JoystickButton rs = new JoystickButton(xbox, XboxController.Button.kStickRight.value);
    public final static ButtonCombination back_start = new ButtonCombination(xbox, XboxController.Button.kBack.value, XboxController.Button.kStart.value);
    public final static Button povd = new POVButton(xbox, 180);
    public final static Button povr = new POVButton(xbox, 90);
    public final static Button povl = new POVButton(xbox, 270);
    public final static Button povu = new POVButton(xbox, 0);


    public static double getXboxLX() {
        return xbox.getRawAxis(XboxController.Axis.kLeftX.value);
    }

    public static double getXboxLY() {
        return xbox.getRawAxis(XboxController.Axis.kLeftY.value);
    }

    public static double getXboxRX() {
        return xbox.getRawAxis(XboxController.Axis.kRightX.value);
    }

    public static double getXboxRY() {
        return xbox.getRawAxis(XboxController.Axis.kRightY.value);
    }

    public static double getLeftStickForward(){
        return -leftStick.getY();
    }

    public static double getRightStickForward(){
        return -rightStick.getY();
    }
}
