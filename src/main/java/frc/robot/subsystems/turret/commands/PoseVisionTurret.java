package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

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
        Pose2d robotPose = drivetrain.getPose();
        turret.setAngle(Utils.calculateTurretAngle(robotPose, true));
    }

    @Override
    public void end(boolean interrupted) {
        VisionModule.setLEDs(false);
    }
}
