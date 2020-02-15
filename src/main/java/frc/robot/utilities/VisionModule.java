package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.annotation.Nullable;

import static frc.robot.Constants.FieldGeometry.PORT_HEIGHT;
import static frc.robot.Constants.Vision.VISION_MODULE_HOOD_DISTANCE;
import static frc.robot.Constants.Vision.VISION_MODULE_HEIGHT;

public class VisionModule extends SubsystemBase {
    private static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("ps3");
    private static NetworkTableEntry visionAngle = visionTable.getEntry("targetYaw");
    private static NetworkTableEntry visionPose = visionTable.getEntry("targetPose");
    private static NetworkTableEntry visionValid = visionTable.getEntry("isValid");
    private static NetworkTableEntry leds = visionTable.getEntry("leds");
    public static VisionModule INSTANCE = new VisionModule();

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
        Double[] pose = visionPose.getDoubleArray(new Double[]{null, null, null});
        if (pose[0] == null) {
            return null;
        }
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    @Nullable
    public static Double getHoodDistance() {
        Pose2d pose = getPose();
        if (pose == null) return null;
        return Math.sqrt(Math.pow(pose.getTranslation().getX(), 2) - Math.pow(PORT_HEIGHT - VISION_MODULE_HEIGHT, 2)) - VISION_MODULE_HOOD_DISTANCE;
    }

    @Override
    public void periodic() {
        Double distance = getHoodDistance();
        if (distance != null) {
            SmartDashboard.putNumber("visionFrontDistance", getHoodDistance());
        }
    }
}
