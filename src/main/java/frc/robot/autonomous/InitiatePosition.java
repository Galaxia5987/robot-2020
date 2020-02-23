package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utilities.VisionModule;

import java.util.List;

public class InitiatePosition extends CommandBase {
    private final Drivetrain drivetrain;
    private final List<Path> toGenerate;
    private Timer timer = new Timer();
    private static final double POSE_TIMEOUT = 0.7;
    private double sumDistance = 0;
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
        sumDistance = 0;
        counter = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        Double distance = VisionModule.getTargetRawDistance();
        if(distance == null) {
            reset();
            return;
        }
        sumDistance += distance;
        counter++;
    }

    @Override
    public boolean isFinished() {
        return timer.get() > POSE_TIMEOUT && counter != 0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPose(VisionModule.getSimplePoseFromDistance(180, sumDistance / counter));
        for(Path path: toGenerate) {
            path.generate(drivetrain.getPose());
        }
    }
}
