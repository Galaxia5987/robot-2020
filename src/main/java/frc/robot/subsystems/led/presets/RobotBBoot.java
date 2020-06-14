package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commnads.BlinkColor;
import frc.robot.subsystems.led.commnads.DimmingColor;

public class RobotBBoot extends SequentialCommandGroup {
    public RobotBBoot(LED led){
        super(
                new DimmingColor(led, Color.kAqua, 3, 1.5),
                new BlinkColor(led, Color.kAqua, 3, 0.25, 0.4)
        );
    }
}
