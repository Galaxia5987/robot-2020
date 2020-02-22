package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LED;
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.function.Supplier;

public class CountBalls extends CommandBase {
    private final LED led;
    private final Timer timer;
    private final Supplier<Integer> ballCount;

    /**
     * Make the LED strip blink.
     *
     * @param led the LED strip subsystem object
     */
    public CountBalls(LED led, Supplier<Integer> ballCount) {
        addRequirements(led);

        this.led = led;
        this.ballCount = ballCount;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (true){
            led.setColorRatios(
                    new ImmutablePair<Double, Color>(1 - (ballCount.get() / 10.), Color.kBlack),
                    new ImmutablePair<Double, Color>((ballCount.get() / 5.), Color.kYellow),
                    new ImmutablePair<Double, Color>(1 - (ballCount.get() / 10.), Color.kBlack)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
