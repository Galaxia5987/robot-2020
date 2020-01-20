package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.FeedTurret;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.shooter.commands.Wait;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;

import static frc.robot.Constants.Shooter.STOP_SHOOTER;
import static frc.robot.Constants.Turret.STOP_TURRET;

public class AutoShoot extends ParallelCommandGroup {

    public AutoShoot(Turret turret, Shooter shooter, Conveyor conveyor) {
        addRequirements(turret, shooter);
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new TurnTurret(turret, STOP_TURRET),
                new SpeedUp(shooter, STOP_SHOOTER),
                // when the flywheel and the turret are at the target speed and angle start feeding the power cells
                new SequentialCommandGroup(new Wait(shooter, turret), new AutoFeed(conveyor)));
    }

}