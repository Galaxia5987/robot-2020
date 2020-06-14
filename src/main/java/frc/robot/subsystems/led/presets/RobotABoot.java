package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;
import frc.robot.subsystems.led.commnads.BlinkColor;
import frc.robot.subsystems.led.commnads.RotatingColor;
import org.apache.commons.lang3.tuple.ImmutablePair;

public class RobotABoot extends SequentialCommandGroup {
    public RobotABoot(LED led){
        super(
                new RotatingColor(led,
                        LEDUtilities.colorLengths(led.getLength(), new ImmutablePair(5, Color.kLime), new ImmutablePair(1,Color.kBlack)
                        ),
                        0.33,
                        1
                ),
                new BlinkColor(led, Color.kLime, 3, 0.2, 0.2)
        );
    }
}
