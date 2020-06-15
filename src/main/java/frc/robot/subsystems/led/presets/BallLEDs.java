package frc.robot.subsystems.led.presets;


import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;
import frc.robot.subsystems.led.commnads.GenericTimerLED;
import frc.robot.utilities.Utils;
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.function.Supplier;
public class BallLEDs extends GenericTimerLED {

    private final Supplier<Integer> ballCount;
    private final double frequency = 0.5;
    public BallLEDs(LED led, Supplier<Integer> ballCount){
        super(led, 0, 1, true);
        this.ballCount = ballCount;
    }

    @Override
    protected AddressableLEDBuffer tick(double time){
        return LEDUtilities.getSymmetric(LEDUtilities.blendColors(led.getLength(), true,
                new ImmutablePair<>((int)(led.getLength() * ballCount.get() * Utils.floorMod(time, frequency) / frequency / 10) , Color.kYellow),
                new ImmutablePair<>( led.getLength() * ballCount.get() / (2*5) + 1, Color.kBlack),
                new ImmutablePair<>(led.getLength() - 3, Color.kBlack),
                new ImmutablePair<>(led.getLength() - 2, Color.kOrange)
                ), true);
    }
}
