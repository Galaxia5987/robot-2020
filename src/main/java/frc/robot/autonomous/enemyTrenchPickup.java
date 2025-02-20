package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

import static frc.robot.Constants.BACK_BUMPER_TO_CENTER;
import static frc.robot.Constants.FieldGeometry.OUTER_PORT_TO_LINE;

public class enemyTrenchPickup extends SequentialCommandGroup {
    private static final TrajectoryConfig toTrenchConfig =
            new TrajectoryConfig(3, 2.5).setReversed(false);
    private static final TrajectoryConfig toShootingConfig =
            new TrajectoryConfig(3.5, 2.5).setReversed(true);
    private static final TrajectoryConfig toRandevouzConfig = 
            new TrajectoryConfig(1, 1).setReversed(false);
    private static final double SHOOT_TIME = 3;
    private static final double LOAD_TIME = 1;
    public Path toTrench = new Path(toTrenchConfig,
            new Pose2d(Units.feetToMeters(35.563), Units.feetToMeters(24.126), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.feetToMeters(32.999), Units.feetToMeters(24.65), Rotation2d.fromDegrees(159))
    );
    public Path toShooting = new Path(toShootingConfig, new Pose2d(Units.feetToMeters(43.268), Units.feetToMeters(11.374), Rotation2d.fromDegrees(150)));
    public Path toRendezvous = new Path(toRandevouzConfig,
            new Pose2d(Units.feetToMeters(36.475), Units.feetToMeters(12.263), Rotation2d.fromDegrees(166)),
            new Pose2d(Units.feetToMeters(34.138), Units.feetToMeters(15.162), Rotation2d.fromDegrees(114))
    );

    public enemyTrenchPickup(Turret turret, Shooter shooter, Drivetrain drivetrain, Conveyor conveyor, Intake intake) {
        addCommands(new InstantCommand(() -> drivetrain.setPose(new Pose2d(15.98 - (OUTER_PORT_TO_LINE + BACK_BUMPER_TO_CENTER), Units.feetToMeters(24.5), Rotation2d.fromDegrees(180)))));
        addCommands(
                new ParallelDeadlineGroup(new FollowPath(drivetrain, toTrench),
                new SequentialCommandGroup(new WaitCommand(0.5), new PickupBalls(intake, conveyor)))
        );
        addCommands(
                new ParallelDeadlineGroup(new FollowPath(drivetrain, toShooting),
                new ParallelDeadlineGroup(new WaitCommand(LOAD_TIME), new PickupBalls(intake, conveyor)),
                new ShootWarmup(turret, shooter, drivetrain, false))
        );
        addCommands(new ParallelDeadlineGroup(new WaitCommand(SHOOT_TIME), new AutoShoot(turret, shooter, conveyor, intake, drivetrain, new VisionTurret(turret))));
        addCommands(new ParallelDeadlineGroup(new FollowPath(drivetrain, toRendezvous), new PickupBalls(intake, conveyor), new ShootWarmup(turret, shooter, drivetrain)));
        addCommands(new AutoShoot(turret, shooter, conveyor, intake, drivetrain,new VisionTurret(turret)));

    }
}
