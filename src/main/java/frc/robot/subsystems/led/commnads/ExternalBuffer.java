package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.led.LED;

import java.util.function.Supplier;

public class ExternalBuffer extends GenericTimerLED {
    Supplier<AddressableLEDBuffer> buffer;

    /**
     * Control a GenericTimerLED command using a supplier for the buffer.
     * This one goes out to dan!
     * @param led LED subsystem
     * @param buffer buffer supplier
     * @param delay delay to update the LEDs
     * @param timeout timeout to the command
     * @param clearOnEnd blank the LEDs at the end
     */
    public ExternalBuffer(LED led, Supplier<AddressableLEDBuffer> buffer, double delay, double timeout, boolean clearOnEnd) {
        super(led, delay, timeout, clearOnEnd);
        this.buffer = buffer;
    }

    @Override
    protected AddressableLEDBuffer tick(double time) {
        return buffer.get();
    }
}
