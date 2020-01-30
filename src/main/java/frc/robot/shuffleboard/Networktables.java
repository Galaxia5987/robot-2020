package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Networktables {

    /**
     * set the value of an entry in the network table
     *
     * @param entry the entry's name
     * @param value the value to set
     */
    public static void setValue(String table, String entry, Object value) {
        if (getValue(table, entry) != value) {
            NetworkTableInstance.getDefault().getTable(table).getEntry(entry).setValue(value);
        }
    }

    /**
     * set the value of an entry in a known network table
     *
     * @param entry the entry of the variable
     * @param value the value to set
     */
    public static void setValue(NetworkTableEntry entry, Object value) {
        if (getValue(entry) != value){
        entry.setValue(value);
    }}

    /**
     * get the value of an entry in the network table
     *
     * @param entry the entry's name
     */
    public static Object getValue(String table, String entry) {
        return NetworkTableInstance.getDefault().getTable(table).getEntry(entry).getValue();
    }

    /**
     * get the value of an entry in a known network table
     *
     * @param entry the entry of the variable
     */
    public static Object getValue(NetworkTableEntry entry) {
        return entry.getValue();
    }
}
