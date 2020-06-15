package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;

/**
 * Command that sets a dimming color.
 */

public class DimmingColor extends GenericTimerLED {
    private final double frequency; //ms
    private final double value_diff; //The amount the value changes. by default, it dims from the colors value to 0.
    private double[] colorHSV;
    public DimmingColor(LED led, Color color, double frequency, double timeout, double value_range, boolean clearOnEnd) {
        super(led, 0, timeout, clearOnEnd);
        this.frequency = frequency;
        colorHSV = HSV.Color2hsv(color);
        this.value_diff = value_range;
    }

    public DimmingColor(LED led, Color color, double frequency, double timeout, boolean clearOnEnd){
        this(led, color, frequency, timeout, HSV.Color2hsv(color)[2], clearOnEnd);
    }

    @Override
    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.singleColor(led.getLength(),HSV.hsv2Color(colorHSV[0],colorHSV[1],Math.min(0, (colorHSV[2] - value_diff) + value_diff * (2-Math.cos(2*Math.PI*time/frequency)) )));
    }
}
