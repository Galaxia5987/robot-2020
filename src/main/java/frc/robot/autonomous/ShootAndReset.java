package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.VisionModule;

import java.util.List;

public class ShootAndReset extends ParallelCommandGroup {

    public ShootAndReset(Turret turret, Shooter shooter, Conveyor conveyor, Drivetrain drivetrain, List<Path> toGenerate, double shootTime) {
        addCommands(new ParallelCommandGroup( // Initiate position while shooting balls
                new SequentialCommandGroup(
                        new AutoDelay(),
                        new ParallelDeadlineGroup(
                                new WaitCommand(shootTime),
                                new AutoShoot(turret, shooter, conveyor, drivetrain, new VisionTurret(turret))
                        )
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> VisionModule.setLEDs(true)),
                        new ConditionalCommand(null, new VisionTurret(turret, true), () -> CustomDashboard.getAutoDelay() > 0),
                        new WaitCommand(1),
                        new InitiatePosition(drivetrain, toGenerate, 180)
                )
        ));
    }

}
