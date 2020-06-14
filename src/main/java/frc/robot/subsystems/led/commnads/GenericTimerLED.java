package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;


public class GenericTimerLED extends CommandBase {
    protected final LED led;
    private final Timer timer = new Timer();
    private double lastTick=0;
    protected double timeout = 100;
    protected double delay = 50;
    public GenericTimerLED(LED led){
        this.led = led;
    }

    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.singleColor(22, Color.kBlack);
    }

    @Override
    public void initialize(){
        lastTick = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() >= lastTick+delay){
            lastTick = timer.get();
            led.set(tick(timer.get()));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
