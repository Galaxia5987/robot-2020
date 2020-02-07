package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Extension of the {@link Button} class, triggers a command when an axis' value raises above a height
 *
 * @author Paulo Khayat
 */
public class TriggerButton extends Button {
    private final GenericHID m_joystick;
    private final int m_axis_number;
    private final double m_starting_value;
    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
     *                     etc)
     * @param axisNumber The button number (see {@link GenericHID#getRawButton(int) }
     *
     * @param threshholdValue above this absolute value the command will run.
     */
    public TriggerButton(GenericHID joystick, int axisNumber, double threshholdValue) {
        m_joystick = joystick;
        m_axis_number = axisNumber;
        m_starting_value = threshholdValue;
    }

    public TriggerButton(XboxController xbox, GenericHID.Hand wantedHand, double startingValue) {
        this(xbox, wantedHand == GenericHID.Hand.kLeft ? XboxController.Axis.kLeftTrigger.value : XboxController.Axis.kRightTrigger.value, startingValue); //Allow inputting xbox triggers directly.
    }

    /**
     * Gets the state of the joystick button, if it under or over the threshold.
     *
     * @return The state of the joystick button
     */
    @Override
    public boolean get() {
        return Math.abs(m_joystick.getRawAxis(m_axis_number)) > m_starting_value;
    }
}