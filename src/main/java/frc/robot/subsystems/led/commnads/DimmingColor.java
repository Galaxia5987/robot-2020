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
    private double frequency = 300; //ms
    private double[] color = HSV.Color2hsv(Color.kRed);
    public DimmingColor(LED led) {
        super(led);
        this.timeout = this.timeout;
        this.delay = 20;
    }

    @Override
    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.singleColor(led.getLength(),HSV.hsv2Color(color[0],color[1],color[2] * (2-Math.cos(2*Math.PI*time/frequency))));
    }
}
