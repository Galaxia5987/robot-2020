package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Utils {

    /**
     * set the value of an entry in the network table
     * @param entry
     * @param value
     */
    public static void setValue(String table, String entry, Object value){
        NetworkTableInstance.getDefault().getTable(table).getEntry(entry).setValue(value);
    }

    /**
     * set the value of an entry in a known network table
     * @param entry
     * @param value
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
        return constrain( map(x, in_min, in_max, out_min, out_max), out_min, out_max);
    }

    public static double constrain(double x, double min, double max){
        return Math.max(min, Math.min(x, max));
    }
}
