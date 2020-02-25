package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;

public class LEDUtilitiesTest {

    @Test
    public void blendRatios(){
        LEDUtilities.printBuffer(LEDUtilities.blendColors(22, false,
                new ImmutablePair<>(3, Color.kYellow),
                new ImmutablePair<>(10, Color.kRed),
                new ImmutablePair<>(17, Color.kBlue)


                ));
    }

}
