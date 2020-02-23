package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utilities.VisionModule;

import java.util.List;

public class InitiatePosition extends CommandBase {
    private final Drivetrain drivetrain;
    private final List<Path> toGenerate;
    private Timer timer = new Timer();
    private static final double POSE_TIMEOUT = 0.7;
    private double sumX = 0;
    private double sumY = 0;
    private double sumAngle = 0;
    private double counter = 0;

    public InitiatePosition(Drivetrain drivetrain, List<Path> toGenerate) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.toGenerate = toGenerate;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
        reset();
    }

    public void reset() {
        sumX = 0;
        sumY = 0;
        sumAngle = 0;
        counter = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Pose2d pose =  VisionModule.getRobotPoseSimple(180);
        if(pose == null) {
            reset();
            return;
        }
        sumX += pose.getTranslation().getX();
        sumY += pose.getTranslation().getY();
        sumAngle += pose.getRotation().getDegrees();
        counter++;
    }

    @Override
    public boolean isFinished() {
        return timer.get() > POSE_TIMEOUT && counter != 0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPose(new Pose2d(sumX / counter, sumY / counter, Rotation2d.fromDegrees(sumAngle / counter)));
        for(Path path: toGenerate) {
            path.generate(drivetrain.getPose());
        }
    }
}
