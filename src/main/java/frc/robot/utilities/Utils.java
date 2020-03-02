package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.UtilityFunctions;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;

import static frc.robot.Constants.FieldGeometry.RED_OUTER_POWER_PORT_LOCATION;

public class Utils {

    /**
     * set the value of an entry in the network table
     *
     * @param entry the network table entry's name
     * @param value the value of the entry
     */
    public static void setValue(String table, String entry, Object value) {
        NetworkTableInstance.getDefault().getTable(table).getEntry(entry).setValue(value);
    }

    /**
     * set the value of an entry in a known network table
     *
     * @param entry the network table entry's name
     * @param value the value of the entry
     */
    public static void setValue(NetworkTableEntry entry, double value) {
        entry.setValue(value);
    }


    /**
     * recreates the results of Math.floorMod() for Double type variables.
     * The result is the unsigned remainder of the mod method.
     *
     * @param value the numerator
     * @param mod   the denominator
     * @return the remainder of the division
     */
    public static double floorMod(double value, double mod) {
        value %= mod;
        value += mod;
        value %= mod;
        return value;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double constrainedMap(double x, double in_min, double in_max, double out_min, double out_max) {
        return MathUtil.clamp(map(x, in_min, in_max, out_min, out_max), out_min, out_max);
    }

    public static void configAllFalcons(FalconConfiguration configurations, TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            falcon.configAllSettings(configurations.motorConfigs);
            falcon.configVoltageCompSaturation(configurations.getVoltageCompensationSaturation());
            falcon.setNeutralMode(configurations.getNeutralMode());
            falcon.enableVoltageCompensation(configurations.isEnableVoltageCompensation());
            falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(configurations.isEnableCurrentLimit()
                    , configurations.getSupplyCurrentLimit()
                    , configurations.getThreshHoldCurrent()
                    , configurations.getThreshHoldTime()));
            falcon.config_kP(0, configurations.getPidSet()[0]);
            falcon.config_kI(0, configurations.getPidSet()[1]);
            falcon.config_kD(0, configurations.getPidSet()[2]);
            falcon.config_kF(0, configurations.getPidSet()[3]);

        }
    }

    /**
     * Calculates turret angle to inner or outer port.
     *
     * @param currentPosition Current robot pose
     * @param innerPort       Aim to inner port
     * @return Turret angle
     */
    public static double calculateTurretAngle(Pose2d currentPosition, boolean innerPort) {
        Pose2d targetLocation = UtilityFunctions.getPortLocation(innerPort);
        double deltaY = targetLocation.getTranslation().getY() - currentPosition.getTranslation().getY();
        double deltaX = targetLocation.getTranslation().getX() - currentPosition.getTranslation().getX();
        double angle = Math.toDegrees(currentPosition.getRotation().getRadians() - Math.atan2(deltaY, deltaX));
        if (angle < 0) angle += 360;
        return angle;
    }

    /**
     * Replaces fields between constants classes.
     *
     * @param class1 Original constants class
     */
    public static void replaceFields(Class class1, Class class2) {
        //Loop and replace all fields
        for (Field f : class2.getDeclaredFields()) {
            for (Field f2 : class1.getDeclaredFields()) {
                //Loop and replace all fields
                if (f2.getName().equals(f.getName())) { // If the name is equal perform replacement

                    f2.setAccessible(true);
                    f.setAccessible(true);
                    try {
                        Field modifiersField = Field.class.getDeclaredField("modifiers");
                        modifiersField.setAccessible(true);
                        modifiersField.setInt(f2, f2.getModifiers() & ~Modifier.FINAL);
                        f2.set(null, f.get(null));
                    } catch (IllegalAccessException | NoSuchFieldException e) { // Catch relevant exceptions
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    public static void swapConstants(Class<?> original, Class<?> B) {
        Utils.replaceFields(original, B); // Replace outer constants
        for (Class aClass : original.getDeclaredClasses()) { // Loop constants classes
            // Find the class in B Constants
            Optional<Class<?>> bClass = Arrays.stream(B.getDeclaredClasses()).filter(c -> c.getSimpleName().equals(aClass.getSimpleName())).findAny();
            if (bClass.isEmpty()) continue; // Class isn't present
            Utils.replaceFields(aClass, bClass.get());
        }
    }

    public static Pose2d getRobotPoseFromX(double x, double rotationDegrees) {
        if (!VisionModule.targetSeen()) return null;
        double relativeTurretAngle = -(RobotContainer.turret.getAngle() + VisionModule.getVisionAngle()) - rotationDegrees; //Turret angle and vision angle should be positive, but because the auto works counter clockwize, we flip the value.
        Pose2d newPose = new Pose2d(
                UtilityFunctions.getPortLocation(false).getTranslation().getX() - x,
                UtilityFunctions.getPortLocation(false).getTranslation().getY() - Math.tan(Math.toRadians(relativeTurretAngle)) * x,
                Rotation2d.fromDegrees(rotationDegrees)
        );
        System.out.println(String.format("Reset with: %s | %s | %s | (%s, %s)", RobotContainer.turret.getAngle(), VisionModule.getVisionAngle(), rotationDegrees, newPose.getTranslation().getX(), newPose.getTranslation().getY()));
        return newPose;
    }

    public static Pose2d getSimplePoseFromDistance(double rotationDegrees, double rawDistance) {
        double relativeTurretAngle = rotationDegrees - RobotContainer.turret.getAngle();
        return new Pose2d(
                UtilityFunctions.getPortLocation(false).getTranslation().getX() - Math.cos(Math.toRadians(relativeTurretAngle)) * rawDistance,
                UtilityFunctions.getPortLocation(false).getTranslation().getY() - Math.sin(Math.toRadians(relativeTurretAngle)) * rawDistance,
                Rotation2d.fromDegrees(rotationDegrees)
        );
    }

    /**
     * returns the distance from the robot to the outer port
     *
     * @param robotPose
     * @return
     */
    public static double localizationDistanceToPort(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(RED_OUTER_POWER_PORT_LOCATION.getTranslation());
    }

}
