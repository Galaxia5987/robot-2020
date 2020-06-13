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
    private double frequency; //ms
    private double[] colorHSV;
    public DimmingColor(LED led, Color color, double frequency, double timeout) {
        super(led);
        this.timeout = timeout;
        this.frequency = frequency;
        this.delay = 20;
        colorHSV = HSV.Color2hsv(color);
    }

    @Override
    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.singleColor(led.getLength(),HSV.hsv2Color(colorHSV[0],colorHSV[1],colorHSV[2] * (2-Math.cos(2*Math.PI*time/frequency))));
    }
}
