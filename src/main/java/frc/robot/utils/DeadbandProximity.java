package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class DeadbandProximity {
    private AnalogInput proximity;
    private double minDistance;
    private double maxDistance;
    private boolean ballSensed;

    public DeadbandProximity(int port, double minDistance, double maxDistance) {
        proximity = new AnalogInput(port);
        this.maxDistance = maxDistance;
        this.minDistance = minDistance;
    }

    public void update() {
        if (isBallAway())
            ballSensed = false;
        else if (isBallClose())
            ballSensed = true;
    }

    public double getRaw() {
        return proximity.getVoltage();
    }

    public boolean isBallSensed() {
        return ballSensed;
    }

    /**
     * retrieve whether the {@link #proximity} sense a Power Cell.
     * If you wish to check whether the proximity lost the Power Cell, use {@link #isBallAway()} ()} instead.
     *
     * @return whether the {@link #proximity} sense a Power Cell.
     */
    private boolean isBallClose() {
        return getRaw() > maxDistance;
    }

    private boolean isBallAway() {
        return getRaw() < minDistance;
    }
}
