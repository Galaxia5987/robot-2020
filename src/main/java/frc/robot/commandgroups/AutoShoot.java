package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.shooter.commands.WaitForShootingVision;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;

import static frc.robot.Constants.Shooter.STOP_SHOOTER;
import static frc.robot.Constants.Turret.STOP_TURRET;

public class AutoShoot extends ParallelDeadlineGroup { // TODO check if you can call a command group with timeout, safety feature

    public AutoShoot(Turret turret, Shooter shooter, Conveyor conveyor) {
        super(new SequentialCommandGroup(new WaitForShootingVision(shooter, turret), new AutoFeed(conveyor)));
        addRequirements(turret, shooter);
        addCommands(
                // turn the turret to the setpoint angle
                // ready the flywheel to shoot the balls to the target distance for the desired amount of time
                new TurnTurret(turret, STOP_TURRET),
                new SpeedUp(shooter),
                // when the flywheel and the turret are at the target speed and angle start feeding the power cells
                new SequentialCommandGroup(new WaitForShootingVision(shooter, turret), new AutoFeed(conveyor))

        );
    }

}
