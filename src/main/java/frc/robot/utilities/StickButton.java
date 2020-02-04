package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Extension of the button class, triggers a command when an axis' value raises above a height
 *
 * @author paulo
 */
public class StickButton extends Button {
    private final GenericHID m_joystick;
    private final int m_axis_number;
    private final double m_starting_value;
    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
     *                     etc)
     * @param axisNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public StickButton(GenericHID joystick, int axisNumber, double threshHoldValue) {
        m_joystick = joystick;
        m_axis_number = axisNumber;
        m_starting_value = threshHoldValue;
    }

    public StickButton(XboxController xbox, GenericHID.Hand wantedHand, double startingValue) {
        this(xbox, wantedHand == GenericHID.Hand.kLeft ? 4 : 1, startingValue); //Allow inputting xbox triggers directly. TODO:check raw joystick values
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        return Math.abs(m_joystick.getRawAxis(m_axis_number)) > m_starting_value;
    }
}