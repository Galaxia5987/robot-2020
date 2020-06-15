package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;

public class ClearLEDs extends InstantCommand {
    private final LED led;
    public ClearLEDs(LED led){
        this.led = led;
    }

    @Override
    public void initialize(){
        led.set(LEDUtilities.singleColor(led.getLength(), Color.kBlack));
    }
    
    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
