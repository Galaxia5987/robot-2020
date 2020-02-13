package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants;

public class VisionModule {
    private static NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("ps3");
    private static NetworkTableEntry visionAngle = visionTable.getEntry("targetYaw");
    private static NetworkTableEntry visionPose = visionTable.getEntry("targetPose");
    private static NetworkTableEntry visionValid = visionTable.getEntry("isValid");
    private static NetworkTableEntry leds = visionTable.getEntry("leds");

    /**
     * @return the angle to the target from the vision network table.
     */
    public static double getVisionAngle(){
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

    public static Pose2d getPose() {
        Double[] pose = visionPose.getDoubleArray(new Double[]{});
        return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
    }

    public static double getNormalizeDistance(){
        return Math.pow(Constants.FieldGeometry.PORT_HEIGHT, 2) - Math.pow(Constants.VISION_MODULE_HEIGHT, 2);
    }

}
