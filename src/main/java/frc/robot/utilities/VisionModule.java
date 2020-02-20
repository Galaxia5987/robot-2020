package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.UtilityFunctions;

import javax.annotation.Nullable;

import static frc.robot.Constants.FieldGeometry.*;
import static frc.robot.Constants.Vision.*;

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
    public static Double getVisionDistance() {
        Double targetDistance = getTargetDistance();
        if (targetDistance == null) return null;
        return Math.sqrt(Math.pow(targetDistance, 2) - Math.pow(PORT_HEIGHT - VISION_MODULE_HEIGHT, 2));
    }

    @Nullable
    public static Double getHoodDistance() {
        Double visionDistance = getVisionDistance();
        if (visionDistance == null) return null;
        return visionDistance + VISION_MODULE_HOOD_DISTANCE;
    }

    @Nullable
    public static Double getRobotDistance() {
        Double visionDistance = getVisionDistance();
        if (visionDistance == null) return null;
        double a = VISION_ROTATION_RADIUS + visionDistance;
        double b = ROBOT_TO_TURRET_CENTER;
        return Math.sqrt(a*a + b*b - 2*a*b*Math.cos(Math.toRadians(RobotContainer.turret.getAngle()))); //Cosine law
    }

    @Override
    public void periodic() {
        Double distance = getHoodDistance();
        if (distance != null) {
            SmartDashboard.putNumber("visionHoodDistance", distance);
            SmartDashboard.putNumber("visionTargetDistance", getTargetDistance());
            SmartDashboard.putNumber("visionRobotDistance", getVisionDistance());
        }
        Pose2d robotPose = getRobotPose();
        if (robotPose != null) {
            SmartDashboard.putNumber("visionRobotX", robotPose.getTranslation().getX());
            SmartDashboard.putNumber("visionRobotY", robotPose.getTranslation().getY());
            SmartDashboard.putNumber("visionRobotAngle", robotPose.getRotation().getDegrees());
        }
        CustomDashboard.setHasVision(targetSeen());
    }

    @Nullable
    public static Pose2d getRobotPose() {
        Pose2d visionPose = getPose();
        Double robotDistance = getVisionDistance();
        if (visionPose == null || robotDistance == null) return null;
        return new Pose2d(
                UtilityFunctions.getAlliancePort(false).getTranslation().getX() - visionPose.getRotation().getCos() * robotDistance,
                UtilityFunctions.getAlliancePort(false).getTranslation().getY() + visionPose.getRotation().getSin() * robotDistance,
                Rotation2d.fromDegrees(RobotContainer.turret.getAngle() - visionPose.getRotation().getDegrees())
        );
    }

}
