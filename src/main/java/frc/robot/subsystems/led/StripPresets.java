package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import org.apache.commons.lang3.tuple.ImmutablePair;

public class StripPresets {
    public static final AddressableLEDBuffer RAINBOW;
    public static final AddressableLEDBuffer WAVES;
    static{
        RAINBOW = LEDUtilities.hsvBlendColors(22, true,
                new ImmutablePair<>(0, Color.kRed),
                new ImmutablePair<>(5, HSV.hsv2Color( 180 * 5 / 22., 1, 1)),
                new ImmutablePair<>(11, Color.kCyan),
                new ImmutablePair<>(17, HSV.hsv2Color(180*17/22., 1, 1))
        );

        WAVES = LEDUtilities.blendColors(22, true,
                new ImmutablePair<>(1, Color.kLightBlue),
                new ImmutablePair<>(2, Color.kDarkBlue),
                new ImmutablePair<>(7, Color.kLightBlue),
                new ImmutablePair<>(8, Color.kDarkBlue),
                new ImmutablePair<>(17, Color.kLightBlue),
                new ImmutablePair<>(18, Color.kDarkBlue));
    }
}
