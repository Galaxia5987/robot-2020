package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commandgroups.AutoShoot;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.FollowPath;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.VisionModule;

import java.util.Collections;

import static frc.robot.Constants.Autonomous.MAX_ACCELERATION;
import static frc.robot.Constants.Autonomous.MAX_SPEED;

public class ShootAndDriveForward extends SequentialCommandGroup {
    public static final double SHOOT_TIME = 10;
    public TrajectoryConfig driveBackConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION).setReversed(true);
    public Path driveBack = new Path(driveBackConfig, new Pose2d(Units.feetToMeters(47.937), Units.feetToMeters(2.933), Rotation2d.fromDegrees(180)));

    public ShootAndDriveForward(Turret turret, Shooter shooter, Drivetrain drivetrain, Conveyor conveyor) {
        addCommands(new ShootAndReset(turret, shooter, conveyor, drivetrain, Collections.singletonList(driveBack), SHOOT_TIME));
        addCommands(new FollowPath(drivetrain, driveBack));
    }

}
