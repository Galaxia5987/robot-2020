package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;
import frc.robot.subsystems.led.StripPresets;
import frc.robot.subsystems.led.commnads.GenericTimerLED;
import org.apache.commons.lang3.tuple.ImmutablePair;

import java.util.function.Supplier;

public class ShootBalls extends GenericTimerLED {
    private double pixels_right = 0;
    private final Supplier<Double> shootSpeed;
    private final Supplier<Double> targetSpeed;
    private final Supplier<Double> turretAngle;
    private final double SPEED_CONSTANT = 70;

    public ShootBalls(LED led, Supplier<Double> shootSpeed, Supplier<Double> targetSpeed, Supplier<Double> turretAngle, boolean clearOnEnd) {
        super(led, 0, 1000 * 10, clearOnEnd);
        this.shootSpeed = shootSpeed;
        this.turretAngle = turretAngle;
        this.targetSpeed = targetSpeed;

    }

    @Override
    protected AddressableLEDBuffer tick(double time) { //TODO: improve this INCREDIBLY process intense animation
        pixels_right += shootSpeed.get() / SPEED_CONSTANT;
        return LEDUtilities.rotateStrip(LEDUtilities.getSymmetric(
                LEDUtilities.rotateStrip(
                        LEDUtilities.blendColors(led.getLength(), true,
                                new ImmutablePair<>(1, Color.kLightBlue),
                                new ImmutablePair<>(2, Color.kDarkBlue),
                                new ImmutablePair<>(7, Color.kLightBlue),
                                new ImmutablePair<>(8, Color.kDarkBlue),
                                new ImmutablePair<>(17, Color.kLightBlue),
                                new ImmutablePair<>(18, Color.kDarkBlue)
                        ), ((int)pixels_right % led.getLength())
                ), true), (int) MathUtil.clamp(Math.round(led.getLength() * (Math.IEEEremainder(turretAngle.get(), 360) - Constants.LED.FIRST_LED_ANGLE) / (Constants.LED.LAST_LED_ANGLE - Constants.LED.FIRST_LED_ANGLE)), 0, led.getLength()) - led.getLength()/2
        );
    }
    }
}
