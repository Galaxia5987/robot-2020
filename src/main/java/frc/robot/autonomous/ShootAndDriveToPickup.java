package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

public class ShootAndDriveToPickup extends SequentialCommandGroup {
    public static final double SHOOT_TIME = 8;
    public Path driveBack = new Path(new Pose2d(Units.feetToMeters(38.321), Units.feetToMeters(15.01), Rotation2d.fromDegrees(180)));

    public ShootAndDriveToPickup(Turret turret, Shooter shooter, Drivetrain drivetrain, Conveyor conveyor) {
        addCommands(new ShootAndReset(turret, shooter, conveyor, drivetrain, Collections.emptyList(), SHOOT_TIME));
        addCommands(new FollowPath(drivetrain, driveBack));
    }

}
