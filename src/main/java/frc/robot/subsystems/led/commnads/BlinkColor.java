package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;

import static frc.robot.Constants.LED.BLINK_PAUSE;
import static frc.robot.Constants.LED.TOTAL_BLINKS;

public class BlinkColor extends CommandBase {
    private final LED led;
    private final Color color;
    private final int blinks;
    private final double period;
    private final Timer timer;
    private Color currentColor;
    private int blinksDone;

    /**
     * Make the LED strip blink.
     *
     * @param led the LED strip subsystem object
     * @param color    color to blink
     */

    public BlinkColor(LED led, Color color) {
        this(led, color, TOTAL_BLINKS, BLINK_PAUSE);
    }

    /**
     * Make the LED strip blink.
     *
     * @param led the LED strip subsystem object
     * @param color    color to blink
     * @param blinks   amount of blinks
     * @param period   time between blinks
     */
    public BlinkColor(LED led, Color color, int blinks, double period) {
        this.led = led;
        this.color = color;
        this.blinks = blinks * 2; // The blinks field counts periods where the LEDs are turned off too, so it's doubled.
        this.period = period;

        blinksDone = 0;
        currentColor = color;
        timer = new Timer();
        addRequirements(led);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (timer.hasPeriodPassed(period)) {
            // Switch between showing the color or turning the LED strip off, to make a blink effect.
            currentColor = currentColor == Color.kBlack ? color : Color.kBlack;
            blinksDone++;
        }
    }

    @Override
    public boolean isFinished() {
        return blinksDone >= blinks;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
