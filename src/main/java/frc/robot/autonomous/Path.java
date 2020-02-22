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
    private TrajectoryConfig config = DEFAULT_CONFIG;
    private List<Pose2d> waypoints;
    private Trajectory trajectory;

    Path(Pose2d... points) {
        waypoints = new ArrayList<>(Arrays.asList(points));
    }

    Path(TrajectoryConfig config, Pose2d... points) {
        this(points);
        this.config = config;
    }

    void setConfig(TrajectoryConfig config) {
        this.config = config;
    }

    public void generate(Pose2d currentRobotPose) {
        waypoints.add(0, currentRobotPose);
        for(Pose2d waypoint: waypoints) {
            System.out.println(String.format("W: %s, %s, %s", waypoint.getTranslation().getX(), waypoint.getTranslation().getY(), waypoint.getRotation().getDegrees()));
        }
        this.trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    public boolean hasTrajectory() {
        return trajectory != null;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

}
