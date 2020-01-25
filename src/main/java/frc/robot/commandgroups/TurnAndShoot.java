package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;

import static frc.robot.Constants.Conveyor.OPEN_GATE;

public class TurnAndShoot extends SequentialCommandGroup {
    // for when there is no vision
    public TurnAndShoot(Turret turret, Shooter shooter, Conveyor conveyor, double timeout, double distance, double angle) {
        addRequirements(turret, shooter);
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new ParallelCommandGroup(new TurnTurret(turret, angle), new SpeedUp(shooter, conveyor, distance).withTimeout(timeout)),
                // shoot the balls until the feed command ends
                new FeedTurret(conveyor, OPEN_GATE),
                new RunCommand(shooter::stop)
        );
    }

}
