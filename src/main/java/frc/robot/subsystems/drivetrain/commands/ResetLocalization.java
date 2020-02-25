package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utilities.VisionModule;

public class ResetLocalization extends CommandBase {
    private final Drivetrain drivetrain;
    private Pose2d pose;

    public ResetLocalization(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        pose = null;
    }

    @Override
    public void execute() {
        pose = VisionModule.getRobotPose();
    }

    @Override
    public boolean isFinished() {
        return pose != null;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPose(pose);
    }
}
