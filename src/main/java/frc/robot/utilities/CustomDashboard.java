package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shuffleboard.NTEntry;

public class CustomDashboard extends SubsystemBase {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("dashboard");
    public static NetworkTable booleans = table.getSubTable("booleans");
    public static NTEntry hasVision = new NTEntry(table, "hasVision");
    public static NTEntry speedValid = new NTEntry(table, "speedValid");
    public static NTEntry distanceValid = new NTEntry(table, "distanceValid");
    public static NTEntry ballCount = new NTEntry(table, "ballsCount");
    public static NTEntry time = new NTEntry(table, "time");
    //Booleans
    public static NTEntry intake = new NTEntry(booleans, "intake");
    public static NTEntry gate = new NTEntry(booleans, "gate");
    public static NTEntry climb = new NTEntry(booleans, "climb");
    public static NTEntry shift = new NTEntry(booleans, "shift");
    //Autonomous
    public static NTEntry autonomousModes = new NTEntry(table, "autonomousModes");
    public static NTEntry selectedMode = new NTEntry(table, "selectedMode");

    public static void setHasVision(boolean toggle) {
        hasVision.setValue(toggle);
    }

    public static void setSpeedValid(boolean toggle) {
        speedValid.setValue(toggle);
    }

    public static void setDistanceValid(boolean toggle) {
        distanceValid.setValue(toggle);
    }

    public static void setBallCount(int count) {
        ballCount.setValue(count);
    }

    public static void setIntake(boolean toggle) {
        intake.setValue(toggle);
    }

    public static void setGate(boolean toggle) {
        gate.setValue(toggle);
    }

    public static void setClimb(boolean toggle) {
        climb.setValue(toggle);
    }

    public static void setShift(boolean toggle) {
        shift.setValue(toggle);
    }

    public static void setTime(int seconds) {
        time.setValue(seconds);
    }

    public static void setAutonomousModes(String[] modes) {
        autonomousModes.setValue(modes);
    }

    public static String getSelectedMode() {
        return (String) selectedMode.getValue("");
    }

    static {
        setAutonomousModes(new String[]{"left", "middle", "right"});
    }

    @Override
    public void periodic() {
        CustomDashboard.setTime((int) Timer.getMatchTime());
    }
}
