package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;

/**
 * Automatically speed up and shoot towards the vision target when ready.
 */
public class AutoShoot extends ParallelDeadlineGroup {
    private static double SHOOTER_WAIT = 0.5;

    public AutoShoot(Turret turret, Shooter shooter, Conveyor conveyor) {
        super(new SequentialCommandGroup(
                new FeedTurret(conveyor, shooter::isShooterReady, turret::isTurretReady, shooter::isShooting),
                new WaitCommand(SHOOTER_WAIT)
        ));
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new VisionTurret(turret, true),
                new SpeedUp(shooter)
        );
    }

}
