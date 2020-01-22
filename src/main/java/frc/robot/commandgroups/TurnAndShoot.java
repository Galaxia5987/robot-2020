package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;

public class TurnAndShoot extends SequentialCommandGroup {
    // for when there is no vision
    public TurnAndShoot(Turret turret, Shooter shooter, double timeout) {
        addRequirements(turret, shooter);
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new ParallelCommandGroup(new TurnTurret(turret), new SpeedUp(shooter, timeout)),
                // shoot the balls until the feed command ends
                new ParallelDeadlineGroup(new FeedTurret(), new SpeedUp(shooter))
        );
    }

}
