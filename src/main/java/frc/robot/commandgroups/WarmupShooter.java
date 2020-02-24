package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurretSwitching;

public class WarmupShooter extends ParallelCommandGroup {

    public WarmupShooter(Turret turret, Shooter shooter, Drivetrain drivetrain) {
        addCommands(new TurretSwitching(turret, drivetrain));
        addCommands(new SpeedUp(shooter, drivetrain));
    }
}
