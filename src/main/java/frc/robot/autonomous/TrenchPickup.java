package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.commandgroups.PickupBalls;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.subsystems.turret.commands.TurretSwitching;
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.State;
import frc.robot.utilities.VisionModule;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static frc.robot.Constants.Autonomous.*;

public class TrenchPickup extends SequentialCommandGroup {
    private static final double INTAKE_WAIT = 0.6;
    private static final double SHOOT_TIME = 4;
    private static final double PICKUP_SPEED = 0.6;

    private static final TrajectoryConfig toTrenchConfig =
            new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION)
                    .addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION))
                    .setEndVelocity(PICKUP_SPEED);

    public Path toTrench = new Path(
            toTrenchConfig,
            new Pose2d(Units.feetToMeters(35.201), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180))
    );

    private static final TrajectoryConfig pickupBallsConfig = new TrajectoryConfig(PICKUP_SPEED, MAX_ACCELERATION)
            .setStartVelocity(PICKUP_SPEED);

    public Path pickupBalls = new Path(
            pickupBallsConfig,
            new Pose2d(Units.feetToMeters(27.669), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180))
    );

    private static final TrajectoryConfig toShootingConfig =
            new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION).setReversed(true)
                    .addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION));

    public Path toShooting = new Path(
            toShootingConfig,
            new Pose2d(Units.feetToMeters(35.201), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180))
    );

    private final List<Path> toGenerate = new ArrayList<>(Collections.singletonList(toTrench));

    public TrenchPickup(Shooter shooter, Conveyor conveyor, Turret turret, Drivetrain drivetrain, Intake intake) {
        addCommands(new ParallelCommandGroup( // Initiate position while shooting balls
                new ParallelDeadlineGroup(
                        new WaitCommand(SHOOT_TIME),
                        new AutoShoot(turret, shooter, conveyor)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(0.4),
                        new InitiatePosition(drivetrain, toGenerate)
                )
        ));
        addCommands(new WaitUntilCommand(toTrench::hasTrajectory));
        addCommands(new ParallelDeadlineGroup( // Drive to trench area and pick up balls with intake
                new SequentialCommandGroup(
                        new FollowPath(drivetrain, toTrench),
                        new FollowPath(drivetrain, pickupBalls)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(INTAKE_WAIT),
                        new PickupBalls(intake, conveyor)
                )
        ));
        addCommands(new ParallelDeadlineGroup( // Drive back to shooting while speeding up
                new FollowPath(drivetrain, toShooting),
                new TurretSwitching(turret, drivetrain),
                new SpeedUp(shooter, false),
                new PickupBalls(intake, conveyor)
        ));
        addCommands(new InstantCommand(() -> VisionModule.setLEDs(true)));
        addCommands(new AutoShoot(turret, shooter, conveyor));
    }
}