package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTEntry {
    private final NetworkTableEntry entry;
    private Object lastValue = null;

    public NTEntry(String table, String entry) {
        this.entry = NetworkTableInstance.getDefault().getTable(table).getEntry(entry);
    }

    /**
     * Set the value of the entry in the network table
     * Ensures there was a change to prevent memory leaks in shuffle board
     * @param value the value to set
     */
    public void setValue(Object value) {
        if (lastValue != value) {
            entry.setValue(value);
            lastValue = value;
        }
    }

    /**
     * @return current value of the entry
     */
    public Object getValue() {
        return entry.getValue();
    }
}
