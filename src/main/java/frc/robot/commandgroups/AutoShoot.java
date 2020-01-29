package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.shooter.commands.WaitForShootingVision;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.subsystems.turret.commands.VisionTurret;

/**
 * Automatically speed up and shoot towards the vision target when ready.
 */
public class AutoShoot extends ParallelDeadlineGroup {

    public AutoShoot(Turret turret, Shooter shooter, Conveyor conveyor) {
        super(new SmartFeed(conveyor,shooter, turret));
        addRequirements(turret, shooter);
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new VisionTurret(turret),
                new SpeedUp(shooter, conveyor)
        );
    }

}
