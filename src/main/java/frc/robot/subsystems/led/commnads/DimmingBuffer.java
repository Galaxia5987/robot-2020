package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;

public class DimmingBuffer extends GenericTimerLED {
    private final double frequency;
    private final AddressableLEDBuffer buffer;
    private final double dim_percent;
    /**
     * works similarly to DimmingColor, except more process expensive
     * @param led LED subsystem
     * @param frequency dimming frequency
     * @param timeout timeout of command
     * @param clearOnEnd clear buffer at end
     */
    public DimmingBuffer(LED led, AddressableLEDBuffer buffer, double frequency, double timeout, double dim_percent, boolean clearOnEnd) {
        super(led, 0, timeout, clearOnEnd);
        this.frequency = frequency;
        this.dim_percent = dim_percent;
        this.buffer = buffer;
    }

    @Override
    protected AddressableLEDBuffer tick(double time) {
        return LEDUtilities.dimStrip(buffer, dim_percent + (1-dim_percent) * (2-Math.cos(2*Math.PI*time/frequency)));
    }
}
