package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.shooter.commands.WaitForShootingVision;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.subsystems.turret.commands.VisionTurret;

/**
 * This command group shoots Power Cells to a distance and angle which are predefined.
 * This command will most likely be obsolete after testing, but it allows you to run all of the necessary functions
 * for shooting.
 */
public class TurnAndShoot extends ParallelDeadlineGroup {
    // for when there is no vision
    public TurnAndShoot(Turret turret, Shooter shooter, Conveyor conveyor, double timeout, double distance, double angle) {
        super(new SequentialCommandGroup(new WaitForShootingVision(shooter, turret), new FeedTurret(conveyor)));
        addRequirements(turret, shooter);
        addCommands(
                new TurnTurret(turret, angle),
                new SpeedUp(shooter, conveyor, distance).withTimeout(timeout)
        );
    }

}