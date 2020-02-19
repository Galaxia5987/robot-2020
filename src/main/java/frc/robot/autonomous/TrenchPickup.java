package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.commandgroups.WaitForVision;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeBalls;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.SpeedUp;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.TurnTurret;
import frc.robot.utilities.State;
import frc.robot.utilities.TrajectoryLoader;

import static frc.robot.Constants.Autonomous.MAX_ACCELERATION;
import static frc.robot.Constants.Autonomous.MAX_SPEED;

public class TrenchPickup extends SequentialCommandGroup {
    public static final double INTAKE_WAIT = 1;
    public static final double SHOOTER_WAIT = 1;
    public static Path toTrench = new Path(
            new Pose2d(35.201, 2.199, Rotation2d.fromDegrees(0)),
            new Pose2d(26.219, 2.199, Rotation2d.fromDegrees(0))
    );
    private static final TrajectoryConfig toShootingConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION).setReversed(true);
    public static Path toShooting = new Path(
            toShootingConfig,
            new Pose2d(35.201, 2.199, Rotation2d.fromDegrees(0)),
            new Pose2d(45.476, 5.923, Rotation2d.fromDegrees(0))
    );

    public TrenchPickup(Shooter shooter, Conveyor conveyor, Turret turret, Drivetrain drivetrain, Intake intake) {
        addCommands(new TurnTurret(turret, -180));
        addCommands(new WaitForVision());
        addCommands(new AutoShoot(turret, shooter, conveyor));
        addCommands(new ParallelCommandGroup(
                new FollowPath(drivetrain, toTrench),
                new SequentialCommandGroup(
                        new WaitCommand(INTAKE_WAIT),
                        new IntakeBalls(intake, conveyor)
                )
        ));
        addCommands(new InstantCommand(() -> intake.setPosition(State.CLOSE)));
        addCommands(new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(SHOOTER_WAIT),
                        new SpeedUp(shooter)
                ),
                new FollowPath(drivetrain, toShooting)
        ));
    }
}
