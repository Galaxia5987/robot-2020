package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utilities.VisionModule;

public class InitiatePosition extends CommandBase {
    private final Drivetrain drivetrain;
    private Pose2d pose;
    private Timer timer = new Timer();
    private static final double POSE_TIMEOUT = 0.3;

    public InitiatePosition(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    @Override
    public void execute() {
        this.pose = VisionModule.getRobotPose();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > POSE_TIMEOUT && pose != null;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPose(pose, pose.getRotation());
        VisionModule.setLEDs(false);
        Path.generateAll(drivetrain.getPose());
    }
}
