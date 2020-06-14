package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;

import static frc.robot.Constants.LED.BLINK_PAUSE;
import static frc.robot.Constants.LED.TOTAL_BLINKS;

public class BlinkColor extends CommandBase {
    private final LED led;
    private final Color color;
    private final int blinks;
    private final double onperiod;
    private final double offperiod;
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
        this(led, color, TOTAL_BLINKS, BLINK_PAUSE, BLINK_PAUSE);
    }

    /**
     * Make the LED strip blink.
     *
     * @param led the LED strip subsystem object
     * @param color    color to blink
     * @param blinks   amount of blinks
     * @param offPeriod   time between blinks
     */
    public BlinkColor(LED led, Color color, int blinks, double onPeriod, double offPeriod) {
        this.led = led;
        this.color = color;
        this.blinks = blinks;
        this.onperiod = onPeriod;
        this.offperiod = offPeriod;
        blinksDone = 0;
        currentColor = Color.kBlack;
        timer = new Timer();
        addRequirements(led);
    }

    public BlinkColor(LED led, Color color, int blinks, double period){
        this(led, color, blinks, period, period);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (timer.hasPeriodPassed((currentColor == color ? onperiod : offperiod))) {
            // Switch between showing the color or turning the LED strip off, to make a blink effect.
            currentColor = currentColor == color ? Color.kBlack : color;
            if(currentColor == color)
                blinksDone++;
            led.set(LEDUtilities.singleColor(led.getLength(), currentColor));
        }
    }

    @Override
    public boolean isFinished() {
        return blinksDone >= blinks;
    }
}
