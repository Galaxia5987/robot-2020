package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;

import java.util.List;

import static frc.robot.Constants.BACK_BUMPER_TO_CENTER;
import static frc.robot.Constants.FieldGeometry.OUTER_PORT_TO_LINE;

public class InitiatePosition extends CommandBase {
    private final Drivetrain drivetrain;
    private final List<Path> toGenerate;
    private Pose2d pose;

    public InitiatePosition(Drivetrain drivetrain, List<Path> toGenerate) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.toGenerate = toGenerate;
    }

    @Override
    public void initialize() {
        VisionModule.setLEDs(true);
    }

    @Override
    public void execute() {
        pose = Utils.getRobotPoseFromX(OUTER_PORT_TO_LINE + BACK_BUMPER_TO_CENTER, 180);
    }

    @Override
    public boolean isFinished() {
        return pose != null;
    }

    @Override
    public void end(boolean interrupted) {
        if(pose != null) {
            drivetrain.setPose(pose);
            for (Path path : toGenerate) {
                path.generate(drivetrain.getPose());
            }
        }
    }
}
