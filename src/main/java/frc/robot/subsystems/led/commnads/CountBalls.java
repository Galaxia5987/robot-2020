package frc.robot.subsystems.led.commnads;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.ColorsBuffer;
import frc.robot.subsystems.led.LED;
import javafx.util.Pair;

import java.util.LinkedHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.LED.BLINK_PAUSE;
import static frc.robot.Constants.LED.TOTAL_BLINKS;

public class CountBalls extends CommandBase {
    private final LED led;
    private final ColorsBuffer initialColorsBuffer;
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
        initialColorsBuffer = led.getCurrentBuffer();
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
                    new Pair<Double, Color>(1 - (ballCount.get() / 10.), Color.kBlack),
                    new Pair<Double, Color>((ballCount.get() / 5.), Color.kYellow),
                    new Pair<Double, Color>(1 - (ballCount.get() / 10.), Color.kBlack)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Make the LED strip show the color(s) it showed before this command started.
        led.setColorBuffer(initialColorsBuffer);
    }
}
