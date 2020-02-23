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
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.State;
import frc.robot.utilities.VisionModule;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static frc.robot.Constants.Autonomous.*;

public class TrenchPickup extends SequentialCommandGroup {
    public static final double INTAKE_WAIT = 0.2;
    public Path toTrench = new Path(
            new Pose2d(Units.feetToMeters(35.201), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180))
    );

    private static final TrajectoryConfig pickupBallsConfig = new TrajectoryConfig(1, MAX_ACCELERATION);

    public Path pickupBalls = new Path(
            pickupBallsConfig,
            new Pose2d(Units.feetToMeters(26.219), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180))
    );

    private static final TrajectoryConfig toShootingConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION).setReversed(true);

    public Path toShooting = new Path(
            toShootingConfig,
            new Pose2d(Units.feetToMeters(35.201), Units.feetToMeters(2.199), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.feetToMeters(42.84), Units.feetToMeters(4.959), Rotation2d.fromDegrees(180))
    );

    private final List<Path> toGenerate = new ArrayList<>(Collections.singletonList(toTrench));

    static {
        toShootingConfig.addConstraint(new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCELERATION));
    }

    public TrenchPickup(Shooter shooter, Conveyor conveyor, Turret turret, Drivetrain drivetrain, Intake intake) {
        addCommands(new TurnTurret(turret, 180)); // Turn so we can see vision target
//        addCommands(new VisionTurret(turret, true)); // Align to target
        addCommands(new InitiatePosition(drivetrain, toGenerate));
        addCommands(new ParallelCommandGroup( // Initiate position while shooting balls
                new AutoShoot(turret, shooter, conveyor)
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
                new VisionTurret(turret),
                new SpeedUp(shooter, false),
                new PickupBalls(intake, conveyor)
        ));
        addCommands(new InstantCommand(() -> VisionModule.setLEDs(true)));
        addCommands(new AutoShoot(turret, shooter, conveyor)); // Shoot picked up balls out
    }
}
