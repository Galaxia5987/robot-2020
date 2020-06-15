package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;


public abstract class GenericTimerLED extends CommandBase{
    protected final LED led;
    private final Timer timer = new Timer();
    protected final double timeout;
    protected final double delay;
    protected final boolean clearOnEnd;
    /**
     * Handles the writing of the buffer onto the LED subsystem. all that needs to be done is to override the 'tick(time)' method
     * @param led LED subsystem
     * @param delay delay between writes in milliseconds.
     * @param timeout time to end writing in milliseconds.
     */
    public GenericTimerLED(LED led, double delay, double timeout, boolean clearOnEnd){
        this.led = led;
        this.delay = delay;
        this.timeout = timeout;
        this.clearOnEnd = clearOnEnd;
        addRequirements(led);
    }

    public GenericTimerLED(LED led, double delay, double timeout){
        this(led, delay,timeout, false);
    }

    protected abstract AddressableLEDBuffer tick(double time);

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(delay == 0 || timer.hasPeriodPassed(delay)){
            led.set(tick(timer.get()));
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }

    @Override
    public void end(boolean interrupted){
        if(clearOnEnd)
             led.set(LEDUtilities.singleColor(led.getLength(), Color.kBlack));
//           led.stop();  //TODO: check if its faster to stop and start the buffer

    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
