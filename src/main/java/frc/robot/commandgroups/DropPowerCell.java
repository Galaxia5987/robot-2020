package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.MinimizeConveyor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeBall;

public class DropPowerCell extends ParallelCommandGroup {

    public DropPowerCell(Conveyor conveyor, Intake intake, double speed, double timeout){
        addRequirements(conveyor, intake);
        addCommands(
                new MinimizeConveyor(),
                new OuttakeBall(speed, timeout)
        );
    }
}
