package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;
import static frc.robot.RobotContainer.navx;

public class PoseVisionTurret extends CommandBase {
    private final Turret turret;
    private final Drivetrain drivetrain;

    public PoseVisionTurret(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    @Override
    public void execute() {
        Pose2d robotPose = VisionModule.getRobotPose();
        if(robotPose == null || turret.inDeadZone()) return;
        turret.setAngle(Utils.calculateTurretAngle(robotPose, true));
        drivetrain.setPose(robotPose, new Rotation2d(Math.toRadians(navx.getAngle())));
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
