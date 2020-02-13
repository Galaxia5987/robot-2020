package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public static double getFrontDistance() {
        return Math.sqrt(Math.pow(getPose().getTranslation().getX(), 2) - Math.pow(PORT_HEIGHT - VISION_MODULE_HEIGHT, 2)) - VISION_MODULE_HOOD_DISTANCE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("visionFrontDistance", getFrontDistance());
    }
}
