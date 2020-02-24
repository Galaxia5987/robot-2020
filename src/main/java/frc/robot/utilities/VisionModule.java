package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.UtilityFunctions;

import javax.annotation.Nullable;

import static frc.robot.Constants.Vision.*;

public class VisionModule extends SubsystemBase {
    private static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("ps3");
    private static NetworkTableEntry visionAngle = visionTable.getEntry("targetYaw");
    private static NetworkTableEntry visionPose = visionTable.getEntry("targetPose");
    private static NetworkTableEntry visionValid = visionTable.getEntry("isValid");
    public static NetworkTableEntry leds = visionTable.getEntry("leds");

    private static LinearFilter filter = LinearFilter.movingAverage(10);
    private static Double filteredDistance = null;

    /**
     * @return the angle to the target from the vision network table.
     */
    public static double getVisionAngle() {
        return visionAngle.getDouble(0);
    }

    /**
     * @return whether the vision sees the target
     */
    public static boolean targetSeen() {
        return visionValid.getBoolean(false);
    }

    public static void setLEDs(boolean on) {
        leds.setBoolean(on);
    }

    @Nullable
    public static Pose2d getPose() {
        Double[] pose = visionPose.getDoubleArray(new Double[]{});
        if (pose.length == 0) {
            return null;
        }
        return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[2]));
    }

    /**
     * @return The distance the camera sees, in meters.
     */
    @Nullable
    public static Double getTargetRawDistance() {
        Pose2d pose = getPose();
        if (pose == null) return null;
        return Math.sqrt(Math.pow(pose.getTranslation().getX(), 2) + Math.pow(pose.getTranslation().getY(), 2));
    }

    @Nullable
    public static Double getHoodDistance() {
        if (filteredDistance == null) return null;
        return filteredDistance + VISION_MODULE_HOOD_DISTANCE;
    }

    @Nullable
    public static Double getRobotDistance() {
        if (filteredDistance == null) return null;
        double a = VISION_ROTATION_RADIUS + filteredDistance;
        double b = ROBOT_TO_TURRET_CENTER;
        return Math.sqrt(a * a + b * b - 2 * a * b * Math.cos(Math.toRadians(RobotContainer.turret.getAngle()))); //Cosine law
    }

    @Override
    public void periodic() {
        Double distance = getTargetRawDistance();
        if(distance == null) {
            filteredDistance = null;
        }
        else {
            if (distance >= 0.1) {
                filteredDistance = calculateMovingAverage(distance);
                SmartDashboard.putNumber("FilteredDistance", filteredDistance);
            }
            SmartDashboard.putNumber("VisionDistance", getTargetRawDistance());
        }
        CustomDashboard.setHasVision(targetSeen());
    }

    @Nullable
    public static Pose2d getRobotPose() {
        Pose2d visionPose = getPose();
        Double robotDistance = getRobotDistance();
        if (visionPose == null || robotDistance == null) return null;
        return new Pose2d(
                UtilityFunctions.getPortLocation(false).getTranslation().getX() - visionPose.getRotation().getCos() * robotDistance,
                UtilityFunctions.getPortLocation(false).getTranslation().getY() + visionPose.getRotation().getSin() * robotDistance,
                Rotation2d.fromDegrees(RobotContainer.turret.getAngle() - visionPose.getRotation().getDegrees())
        );
    }

    private static double calculateMovingAverage(double distance) {
        return filter.calculate(distance);
    }
}
