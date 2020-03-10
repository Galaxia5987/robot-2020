package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Ports;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.RotationControl;

public class WaitRotational extends SequentialCommandGroup {
    private ColorWheel colorWheel;

    public WaitRotational(ColorWheel colorWheel){
        colorWheel.updateSensor();
        new SequentialCommandGroup(new WaitCommand(0.04), new RotationControl(colorWheel));
    }


}
