package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class holds the static methods of the buttons, in cases where commands want to read the controller values.
 * This should not be confused with RobotContainer, which sets up autonomous commands and maps commands to buttons.
 */
public class OI {
    //Controllers are defined here. (Only the RobotContainer and OI should access them)
    final static Joystick leftStick = new Joystick(0);
    final static Joystick rightStick = new Joystick(1);
    final static XboxController xbox = new XboxController(2);

    public static double getXboxY(){
        return xbox.getY(GenericHID.Hand.kRight);
    }
}
