package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;

public class RotatingColor extends GenericTimerLED {
    private double frequency;
    private AddressableLEDBuffer strip;
    public RotatingColor(LED led, AddressableLEDBuffer strip, double frequency, double timeout) {
        super(led);
        this.timeout = timeout;
        this.frequency = frequency;
        this.delay = 20;
        this.strip = strip;
    }

    @Override
    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.getShifted(strip, (int) (22* time / frequency));
        //return LEDUtilities.getShifted(strip, (int) (22* (2-Math.cos(2*Math.PI*time/frequency))));
    }
}

