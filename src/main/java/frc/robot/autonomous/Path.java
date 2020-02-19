package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static frc.robot.Constants.Autonomous.MAX_ACCELERATION;
import static frc.robot.Constants.Autonomous.MAX_SPEED;

public class Path {
    private static final TrajectoryConfig DEFAULT_CONFIG = new TrajectoryConfig(MAX_SPEED, MAX_ACCELERATION);
    private static final List<Path> pathRegistry = new ArrayList<>();

    private TrajectoryConfig config = DEFAULT_CONFIG;
    private final List<Pose2d> waypoints = new ArrayList<>();
    private Trajectory trajectory;

    public static void generateAll(Pose2d robotPose) {
        for (Path path : pathRegistry) {
            path.generate(robotPose);
        }
    }

    Path(Pose2d... points) {
        waypoints.addAll(Arrays.asList(points));
        pathRegistry.add(this);
    }

    Path(TrajectoryConfig config, Pose2d... points) {
        this(points);
        this.config = config;
    }

    void setConfig(TrajectoryConfig config) {
        this.config = config;
    }

    void generate(Pose2d currentRobotPose) {
        waypoints.add(0, currentRobotPose);
        this.trajectory = TrajectoryGenerator.generateTrajectory(
                waypoints,
                config
        );
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}
