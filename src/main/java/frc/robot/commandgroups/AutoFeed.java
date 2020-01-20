package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.conveyor.commands.Gate;

import static frc.robot.Constants.Conveyor.OPEN_GATE;

public class AutoFeed extends ParallelCommandGroup {

    public AutoFeed(Conveyor conveyor){
        addRequirements(conveyor);
        addCommands(
                // move the gate and let the power cells move into the turret
                new Gate(conveyor, OPEN_GATE),
                // feed the power cells to the turret
                new FeedTurret()
        );
    }
}
