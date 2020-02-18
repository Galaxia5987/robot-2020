package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CustomDashboard extends SubsystemBase {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("dashboard");
    public static NetworkTable booleans = table.getSubTable("booleans");
    public static NetworkTableEntry hasVision = table.getEntry("hasVision");
    public static NetworkTableEntry speedValid = table.getEntry("speedValid");
    public static NetworkTableEntry distanceValid = table.getEntry("distanceValid");
    public static NetworkTableEntry ballCount = table.getEntry("ballsCount");
    public static NetworkTableEntry time = table.getEntry("time");
    //Booleans
    public static NetworkTableEntry intake = booleans.getEntry("intake");
    public static NetworkTableEntry gate = booleans.getEntry("gate");
    public static NetworkTableEntry climb = booleans.getEntry("climb");
    public static NetworkTableEntry shift = booleans.getEntry("shift");
    //Autonomous
    public static NetworkTableEntry autonomousModes = table.getEntry("autonomousModes");
    public static NetworkTableEntry selectedMode = table.getEntry("selectedMode");

    public static void setHasVision(boolean toggle) {
        hasVision.setBoolean(toggle);
    }

    public static void setSpeedValid(boolean toggle) {
        speedValid.setBoolean(toggle);
    }

    public static void setDistanceValid(boolean toggle) {
        distanceValid.setBoolean(toggle);
    }

    public static void setBallCount(int count) {
        ballCount.setNumber(count);
    }

    public static void setIntake(boolean toggle) {
        intake.setBoolean(toggle);
    }

    public static void setGate(boolean toggle) {
        gate.setBoolean(toggle);
    }

    public static void setClimb(boolean toggle) {
        climb.setBoolean(toggle);
    }

    public static void setShift(boolean toggle) {
        shift.setBoolean(toggle);
    }

    public static void setTime(int seconds) {
        time.setNumber(seconds);
    }

    public static void setAutonomousModes(String[] modes) {
        autonomousModes.setStringArray(modes);
    }

    public static String getSelectedMode() {
        return selectedMode.getString("");
    }

    static {
        setAutonomousModes(new String[]{"left", "middle", "right"});
    }

    @Override
    public void periodic() {
        CustomDashboard.setTime((int) Timer.getMatchTime());
    }
}
