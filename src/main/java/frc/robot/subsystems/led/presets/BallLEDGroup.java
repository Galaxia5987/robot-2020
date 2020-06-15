package frc.robot.subsystems.led.presets;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.commnads.DimmingColor;

import java.util.function.BooleanSupplier;

public class BallLEDGroup extends ConditionalCommand {

    public BallLEDGroup(LED led, Conveyor conveyor) {
        super(
                new DimmingColor(led, Color.kYellow, 0.25, 1, 45, true),
                new BallLEDs(led, conveyor::getBallsCount),
                () -> conveyor.getBallsCount() == Constants.Conveyor.MAX_BALLS_AMOUNT
        );
    }
}
