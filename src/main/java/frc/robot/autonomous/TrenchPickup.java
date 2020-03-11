package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.commandgroups.ShootWarmup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.VisionModule;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static frc.robot.Constants.Autonomous.*;
import static frc.robot.Constants.Conveyor.FUNNEL_INTAKE_POWER;

public class TrenchPickup extends SequentialCommandGroup {
    private static final double INTAKE_WAIT = 1.2;
    private static final double SHOOT_TIME = 4;
    private static final double TRENCH_SHOOT = 3;
    private static final double PICKUP_SPEED = 2.0;

    private static final TrajectoryConfig toTrenchConfig =
            new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION)
                    .addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION))
                    .setEndVelocity(PICKUP_SPEED);

    public Path toTrench = new Path(
            toTrenchConfig,
            new Pose2d(Units.feetToMeters(35.201), Units.feetToMeters(2.3), Rotation2d.fromDegrees(180))
    );

    private static final TrajectoryConfig pickupBallsConfig = new TrajectoryConfig(PICKUP_SPEED, MAX_ACCELERATION)
            .setStartVelocity(PICKUP_SPEED);

    private static final TrajectoryConfig randezvouzConfig = new TrajectoryConfig(1, 1)
            .setStartVelocity(0);

    public Path pickupBalls = new Path(
            pickupBallsConfig,
            new Pose2d(Units.feetToMeters(26.5), Units.feetToMeters(2.3), Rotation2d.fromDegrees(180))
    );

    public Path randezvouzPickup = new Path(randezvouzConfig
            , new Pose2d(Units.feetToMeters(33.03), Units.feetToMeters(8.627), Rotation2d.fromDegrees(110)));

    private static final TrajectoryConfig toShootingConfig =
            new TrajectoryConfig(3.5, 2.5).setReversed(true);

    public Path toShooting = new Path(
            toShootingConfig,
            new Pose2d(Units.feetToMeters(36.182), Units.feetToMeters(4.705), Rotation2d.fromDegrees(166))
    );

    private final List<Path> toGenerate = new ArrayList<>(Collections.singletonList(toTrench));

    public TrenchPickup(Shooter shooter, Conveyor conveyor, Turret turret, Drivetrain drivetrain, Intake intake) {
        addCommands(new ShootAndReset(turret, shooter, conveyor, drivetrain, toGenerate, SHOOT_TIME));
        addCommands(new WaitUntilCommand(toTrench::hasTrajectory));
        addCommands(new ParallelDeadlineGroup( // Drive to trench area and pick up balls with intake
                new SequentialCommandGroup(
                        new FollowPath(drivetrain, toTrench),
                        new ParallelDeadlineGroup(
                                new FollowPath(drivetrain, pickupBalls),
                                new ShootWarmup(turret, shooter, drivetrain, false)
                        )
                ),
                new SequentialCommandGroup(
                        new WaitCommand(INTAKE_WAIT),
                        new PickupBalls(intake, conveyor)
                )
        ));
        addCommands(new FollowPath(drivetrain, toShooting));
        addCommands(new ParallelDeadlineGroup(new FollowPath(drivetrain, randezvouzPickup),
                new SequentialCommandGroup(new WaitCommand(INTAKE_WAIT),
                        new PickupBalls(intake, conveyor))),
                new ShootWarmup(turret, shooter, drivetrain));
        addCommands(new InstantCommand(() -> VisionModule.setLEDs(true)));
        addCommands(new ParallelDeadlineGroup(new WaitCommand(TRENCH_SHOOT),
                new AutoShoot(turret, shooter, conveyor, drivetrain, new VisionTurret(turret))));
    }
}
