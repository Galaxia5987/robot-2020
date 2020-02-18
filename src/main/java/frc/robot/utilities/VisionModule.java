package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import javax.annotation.Nullable;

import static frc.robot.Constants.FieldGeometry.*;
import static frc.robot.Constants.Vision.VISION_MODULE_HOOD_DISTANCE;
import static frc.robot.Constants.Vision.VISION_MODULE_HEIGHT;

public class VisionModule extends SubsystemBase {
    private static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("ps3");
    private static NetworkTableEntry visionAngle = visionTable.getEntry("targetYaw");
    private static NetworkTableEntry visionPose = visionTable.getEntry("targetPose");
    private static NetworkTableEntry visionValid = visionTable.getEntry("isValid");
    private static NetworkTableEntry leds = visionTable.getEntry("leds");

    /**
     * @return the angle to the target from the vision network table.
     */
    public static double getVisionAngle() {
        return visionAngle.getDouble(-100);
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

    @Nullable
    public static Double getTargetDistance() {
        Pose2d pose = getPose();
        if (pose == null) return null;
        return Math.sqrt(Math.pow(pose.getTranslation().getX(), 2) + Math.pow(pose.getTranslation().getY(), 2));
    }

    @Nullable
    public static Double getRobotDistance() {
        Double targetDistance = getTargetDistance();
        if(targetDistance == null) return null;
        return Math.sqrt(Math.pow(targetDistance, 2) - Math.pow(PORT_HEIGHT - VISION_MODULE_HEIGHT, 2));
    }

    @Nullable
    public static Double getHoodDistance() {
        Double robotDistance = getRobotDistance();
        if (robotDistance == null) return null;
        return robotDistance + VISION_MODULE_HOOD_DISTANCE;
    }

    @Override
    public void periodic() {
        Double distance = getHoodDistance();
        if (distance != null) {
            SmartDashboard.putNumber("visionHoodDistance", distance);
            SmartDashboard.putNumber("visionTargetDistance", getTargetDistance());
            SmartDashboard.putNumber("visionRobotDistance", getRobotDistance());
        }
        Pose2d robotPose = getRobotPose();
        if(robotPose != null) {
            SmartDashboard.putNumber("visionRobotX", robotPose.getTranslation().getX());
            SmartDashboard.putNumber("visionRobotY", robotPose.getTranslation().getY());
            SmartDashboard.putNumber("visionRobotAngle", robotPose.getRotation().getDegrees());
        }
        CustomDashboard.setHasVision(targetSeen());
    }

    @Nullable
    public static Pose2d getRobotPose() {
        Pose2d visionPose = getPose();
        Double robotDistance = getRobotDistance();
        if(visionPose == null || robotDistance == null) return null;
        return new Pose2d(
                OUTER_POWER_PORT_LOCATION.getTranslation().getX() - visionPose.getRotation().getCos() * robotDistance,
                OUTER_POWER_PORT_LOCATION.getTranslation().getY() - visionPose.getRotation().getSin() * robotDistance,
                Rotation2d.fromDegrees(visionPose.getRotation().getDegrees() - RobotContainer.turret.getAngle())
        );
    }

}
