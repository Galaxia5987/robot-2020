package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.text.DecimalFormat;

public class CustomDashboard extends SubsystemBase {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("dashboard");
    private static NetworkTable booleans = table.getSubTable("booleans");
    private static NetworkTable values = table.getSubTable("values");

    private static NetworkTableEntry hasVision = table.getEntry("hasVision");
    private static NetworkTableEntry speedValid = table.getEntry("speedValid");
    private static NetworkTableEntry turretReady = table.getEntry("turretReady");
    private static NetworkTableEntry ballCount = table.getEntry("ballsCount");
    private static NetworkTableEntry time = table.getEntry("time");
    private static NetworkTableEntry autoDelay = table.getEntry("autoDelay");

    //Booleans
    private static NetworkTableEntry climb = booleans.getEntry("climb");
    private static NetworkTableEntry shift = booleans.getEntry("shift");
    private static NetworkTableEntry CWColor = table.getEntry("CWColor");

    //Autonomous
    private static NetworkTableEntry autonomousModes = table.getEntry("autonomousModes");
    private static NetworkTableEntry selectedMode = table.getEntry("selectedMode");

    //Values
    private static NetworkTableEntry gyroRoll = values.getEntry("gyroRoll");
    private static NetworkTableEntry turretAngle = values.getEntry("turretAngle");
    private static NetworkTableEntry climbLeftHeight = values.getEntry("climbLeftHeight");
    private static NetworkTableEntry climbRightHeight = values.getEntry("climbRightHeight");

    public static void setHasVision(boolean toggle) {
        hasVision.setBoolean(toggle);
    }

    public static void setSpeedValid(boolean toggle) {
        speedValid.setBoolean(toggle);
    }

    public static void setTurretReady(boolean toggle) {
        turretReady.setBoolean(toggle);
    }

    public static void setBallCount(int count) {
        ballCount.setNumber(count);
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

    public static void setGyroRoll(float roll) {
        gyroRoll.setDouble(truncateDecimal(roll));
    }

    public static void setClimbLeftHeight(double height) {
        climbLeftHeight.setDouble(truncateDecimal(height));
    }

    public static void setClimbRightHeight(double height) {
        climbRightHeight.setDouble(truncateDecimal(height));
    }

    public static void setTurretAngle(double angle) {
        turretAngle.setDouble(angle);
    }

    public static double getAutoDelay() {
        return autoDelay.getDouble(0);
    }

    public static String getSelectedMode() {
        return selectedMode.getString("");
    }

    public static void setCWColor() {
        String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
        if (gameMessage.isEmpty()) return;
        if (gameMessage.charAt(0) == 'Y') CWColor.setString("yellow");
        if (gameMessage.charAt(0) == 'R') CWColor.setString("red");
        if (gameMessage.charAt(0) == 'G') CWColor.setString("green");
        if (gameMessage.charAt(0) == 'B') CWColor.setString("blue");
    }

    @Override
    public void periodic() {
        setTime((int) Timer.getMatchTime());
        setCWColor();
        setGyroRoll(RobotContainer.navx.getRoll());
    }

    public static double truncateDecimal(double number) {
        return Math.floor(number * 100) / 100;
    }
}
