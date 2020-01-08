package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.subsystems.turret.Turret.table;

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

}
