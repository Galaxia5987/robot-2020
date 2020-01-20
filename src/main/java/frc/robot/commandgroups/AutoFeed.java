package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.Gate;

public class AutoFeed extends ParallelCommandGroup {

    public AutoFeed(Conveyor conveyor, boolean gate){
        addRequirements(conveyor);
        addCommands(
                new FeedTurret(),
                new Gate(conveyor, gate)
        );
    }
}
