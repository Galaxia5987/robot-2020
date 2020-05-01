package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;

import java.util.Arrays;

public class LEDUtilitiesTest {

    @Test
    public void blendRatios(){
        LEDUtilities.printHSV(LEDUtilities.colorLengths(22,
                new ImmutablePair<>(5, edu.wpi.first.wpilibj.util.Color.kRed),
                new ImmutablePair<>(6, Color.kLime),
                new ImmutablePair<>(5, Color.kCyan),
                new ImmutablePair<>(6, Color.kPurple)));
        System.out.println(Arrays.toString(HSV.hsv2rgb(HSV.rgb2hsv(0, 1, 1)[0], HSV.rgb2hsv(0, 1, 1)[1],1)));

    }

}
