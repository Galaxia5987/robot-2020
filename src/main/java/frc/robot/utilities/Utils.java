package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.UtilityFunctions;

import static frc.robot.Constants.FieldGeometry.RED_INNER_POWER_PORT_LOCATION;
import static frc.robot.Constants.FieldGeometry.RED_OUTER_POWER_PORT_LOCATION;

public class Utils {

    /**
     * set the value of an entry in the network table
     * @param entry the network table entry's name
     * @param value the value of the entry
     */
    public static void setValue(String table, String entry, Object value){
        NetworkTableInstance.getDefault().getTable(table).getEntry(entry).setValue(value);
    }

    /**
     * set the value of an entry in a known network table
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
     * @param mod the denominator
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
     * @param currentPosition Current robot pose
     * @param innerPort Aim to inner port
     * @return Turret angle
     */
    public static double calculateTurretAngle(Pose2d currentPosition, boolean innerPort) {
        Pose2d targetLocation = UtilityFunctions.getAlliancePort(innerPort);
        double deltaY = targetLocation.getTranslation().getY() - currentPosition.getTranslation().getY();
        double deltaX = targetLocation.getTranslation().getX() - currentPosition.getTranslation().getX();
        return Math.toDegrees(Math.atan2(deltaY, deltaX) - currentPosition.getRotation().getRadians());
    }
}
