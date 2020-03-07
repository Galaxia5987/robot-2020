package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.commandgroups.ShootWarmup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;

public class enemyTrenchPickup extends SequentialCommandGroup {
    private static final TrajectoryConfig toShootingConfig =
            new TrajectoryConfig(3.5, 2.5).setReversed(true);
    private static final TrajectoryConfig toTrenchConfig =
            new TrajectoryConfig(3.5, 2.5).setReversed(false);
    public Path driveToTrench = new Path(toTrenchConfig, new Pose2d(Units.feetToMeters(33.89), Units.feetToMeters(24.5), Rotation2d.fromDegrees(0)));
    public Path toShooting = new Path(toShootingConfig, new Pose2d(Units.feetToMeters(42.9), Units.feetToMeters(11), Rotation2d.fromDegrees(0)));


    public enemyTrenchPickup(Turret turret, Shooter shooter, Drivetrain drivetrain, Conveyor conveyor, Intake intake) {
        addCommands(new InstantCommand(() -> drivetrain.setPose(new Pose2d(Units.feetToMeters(43.1), Units.feetToMeters(18.9), Rotation2d.fromDegrees(180)))));
        addCommands(new ParallelDeadlineGroup(new FollowPath(drivetrain, driveToTrench),
                new SequentialCommandGroup(new WaitCommand(1.5),
                        new PickupBalls(intake, conveyor))));
        addCommands(new ParallelDeadlineGroup(new FollowPath(drivetrain, toShooting),
                new ShootWarmup(turret, shooter, drivetrain, false)));
        addCommands(new AutoShoot(turret, shooter, conveyor, drivetrain, new VisionTurret(turret)));
    }
}
