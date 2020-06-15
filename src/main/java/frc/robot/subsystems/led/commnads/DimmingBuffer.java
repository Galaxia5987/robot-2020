package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LED;

public class DimmingBuffer extends GenericTimerLED {
    private final double frequency;
    private final int[][] bufferHSV;
    private final double value_diff;
    /**
     * works similarly to DimmingColor, except more process expensive
     * @param led LED subsystem
     * @param frequency dimming frequency
     * @param timeout timeout of command
     * @param clearOnEnd clear buffer at end
     */
    public DimmingBuffer(LED led, AddressableLEDBuffer buffer, double frequency, double timeout, double value_diff, boolean clearOnEnd) {
        super(led, 0, timeout, clearOnEnd);
        this.frequency = frequency;
        this.value_diff = value_diff;
        bufferHSV = new int[buffer.getLength()][3];
        for(int i = 0; i < buffer.getLength(); i++ ){
            double[] hsv = HSV.Color2hsv(buffer.getLED(i));
            bufferHSV[i] = new int[]{(int) hsv[0], (int) hsv[1], (int) hsv[2]};
        }
    }

    @Override
    protected AddressableLEDBuffer tick(double time) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(led.getLength());
        for(int i = 0; i < led.getLength(); i++){
            buffer.setHSV(i, bufferHSV[i][0], bufferHSV[i][1], (int) Math.min(0, (bufferHSV[i][2] - value_diff + value_diff * (2-Math.cos(2*Math.PI*time/frequency)))));
        }
        return buffer;
    }
}
