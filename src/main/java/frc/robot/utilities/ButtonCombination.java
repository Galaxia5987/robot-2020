package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Extension of the button class, triggers a command when all of the buttons are pressed simultaneously
 *
 * @author Paulo Khayat
 */
public class ButtonCombination extends Button {

    private final GenericHID m_joystick;
    private final int[] buttons;
    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
     *                     etc)
     * @param buttons The button IDS  which need to all be pressed (see {@link GenericHID#getRawButton(int) }
     */
    public ButtonCombination(GenericHID joystick, int ... buttons) {
        this.buttons = buttons;
        m_joystick = joystick;
    }
    @Override
    public boolean get() {
        for (int button : buttons){
            if (!m_joystick.getRawButton(button))
                return false;
        }
        return true;
    }
}