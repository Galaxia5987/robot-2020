package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;

import java.util.Collections;

public class ShootAndDriveToPickup extends SequentialCommandGroup {
    public static final double SHOOT_TIME = 10;
    public Path driveBack = new Path(new Pose2d(Units.feetToMeters(36.864), Units.feetToMeters(17.398), Rotation2d.fromDegrees(180)));

    public ShootAndDriveToPickup(Turret turret, Shooter shooter, Drivetrain drivetrain, Conveyor conveyor) {
        addCommands(new ParallelCommandGroup( // Initiate position while shooting balls
                new ParallelDeadlineGroup(
                        new WaitCommand(SHOOT_TIME),
                        new AutoShoot(turret, shooter, conveyor, drivetrain, new VisionTurret(turret))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(0.4),
                        new InitiatePosition(drivetrain, Collections.singletonList(driveBack), 180)
                )
        ));
        addCommands(new FollowPath(drivetrain, driveBack));
    }

}
