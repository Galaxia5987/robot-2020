package frc.robot.utilities;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

public class TrajectoryLoader {
    private static Map<String, Trajectory> trajectories = new ConcurrentHashMap<>();

    static {
        for (File file : Objects.requireNonNull(new File(Filesystem.getDeployDirectory() + "/trajectories").listFiles())) {
            try {
                trajectories.put(file.getName(), TrajectoryUtil.fromPathweaverJson(file.toPath()));
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public Trajectory getTrajectory(String name) {
        return trajectories.get(name);
    }


}
