package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.AddressableLEDBuffer;
import frc.robot.subsystems.led.LED;

import static frc.robot.Constants.LED.BLINK_PAUSE;
import static frc.robot.Constants.LED.TOTAL_BLINKS;

public class BlinkColor extends CommandBase {
    private final LED ledStrip;
    private final Color color;
    private final int blinks;
    private final double period;
    private final Timer timer;
    private final AddressableLEDBuffer initialColorsBuffer;
    private Color currentColor;
    private int blinksDone;

    /**
     * Make the LED strip blink.
     *
     * @param ledStrip the LED strip subsystem object
     * @param color    color to blink
     */

    public BlinkColor(LED ledStrip, Color color) {
        this(ledStrip, color, TOTAL_BLINKS, BLINK_PAUSE);
    }

    /**
     * Make the LED strip blink.
     *
     * @param ledStrip the LED strip subsystem object
     * @param color    color to blink
     * @param blinks   amount of blinks
     * @param period   time between blinks
     */
    public BlinkColor(LED ledStrip, Color color, int blinks, double period) {
        this.ledStrip = ledStrip;
        this.color = color;
        this.blinks = blinks * 2; // The blinks field counts periods where the LEDs are turned off too, so it's doubled.
        this.period = period;

        this.blinksDone = 0;
        currentColor = color;
        timer = new Timer();
        this.initialColorsBuffer = ledStrip.getCurrentBuffer();
        addRequirements(ledStrip);
    }

    @Override
    public void initialize() {
        timer.start();
        ledStrip.setWholeStrip(color);
    }

    @Override
    public void execute() {
        if (timer.hasPeriodPassed(period)) {
            // Switch between showing the color or turning the LED strip off, to make a blink effect.
            currentColor = currentColor == Color.kBlack ? color : Color.kBlack;
            ledStrip.setWholeStrip(currentColor);
            blinksDone++;
        }
    }

    @Override
    public boolean isFinished() {
        return blinksDone >= blinks;
    }

    @Override
    public void end(boolean interrupted) {
        // Make the LED strip show the color(s) it showed before this command started.
        ledStrip.setColorBuffer(initialColorsBuffer);
    }
}
