package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.led.HSV;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDUtilities;
import frc.robot.subsystems.led.commnads.DimmingColor;
import frc.robot.subsystems.led.commnads.GenericTimerLED;

import static frc.robot.Constants.LED.DEFAULT_COLOR;

//Although identical to the dimming color class, i chose to create a separate copy of the dimming leds so that the colors could change in real time.
public class IdleRobot extends ConditionalCommand {
    public IdleRobot(LED led){
        super(
                new DimmingColor(led, HSV.hsv2Color(122, 255, 75), 5, 99999), //TODO: replace hsv2Color with BLUE
                new ConditionalCommand(
                        new DimmingColor(led, HSV.hsv2Color(0, 255, 75), 1, 99999, 30), //TODO: replace hsv2Color with RED
                        new DimmingColor(led, DEFAULT_COLOR, 1, 99999),
                        () -> DriverStation.getInstance().getAlliance().equals(DriverStation.Alliance.Red)
                ),
                () -> DriverStation.getInstance().getAlliance().equals(DriverStation.Alliance.Blue)
        );
    }
}
