package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.WaitForShootingVision;
import frc.robot.subsystems.turret.Turret;

public class SmartFeed extends SequentialCommandGroup {

    public SmartFeed(Conveyor conveyor, Shooter shooter, Turret turret) {
        addRequirements(conveyor, shooter, turret);
        addCommands(
                // waits for the shooter to speed up and releases the balls one at a time
                new WaitForShootingVision(shooter, turret),
                new FeedTurret(conveyor, shooter)
        );
    }
}
