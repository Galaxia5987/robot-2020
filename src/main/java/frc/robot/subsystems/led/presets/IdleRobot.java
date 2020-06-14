package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;
import frc.robot.subsystems.led.commnads.GenericTimerLED;
//Although identical to the dimming color class, i chose to create a separate copy of the dimming leds so that the colors could change in real time.
public class IdleRobot extends GenericTimerLED {
    public IdleRobot(LED led){
        super(led);
        this.timeout = 99999;
    }

    @Override
    protected AddressableLEDBuffer tick(double time) {
        int hue;
        switch(DriverStation.getInstance().getAlliance()){
            case Red:
                hue = 0;
                break;
            case Blue:
                hue = 122;
                break;
            default:
                hue = 95;
        }
        return LEDUtilities.singleColor(led.getLength(), HSV.hsv2Color(hue, 255,45 + 30 * (2-Math.cos(2*Math.PI*time/1))));

    }
}
