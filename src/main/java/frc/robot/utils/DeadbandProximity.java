package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class DeadbandProximity {
    private AnalogInput proximity;
    private double minDistance;
    private double maxDistance;
    private boolean ballSensed = false;

    public DeadbandProximity(int port, double minDistance, double maxDistance) {
        proximity = new AnalogInput(port);
        this.maxDistance = maxDistance;
        this.minDistance = minDistance;
    }

    /**
     * check whether the ballSensed was sensed by the proximity.
     */
    public void update() {
        if (isBallAway()) {
            ballSensed = false;
        }else if (isBallClose() && !ballSensed) {
            ballSensed = true;
        }
    }

    /**
     * Retrieve the proximity's voltage.
     *
     * @return the proximity's voltage.
     */
    public double getRaw() {
        return proximity.getVoltage();
    }

    /**
     * Retrieve whether the ball was sensed by the proximity.
     *
     * @return whether the ball was sensed by the proximity.
     */
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

    /**
     * retrieve whether the {@link #proximity} lost the Power Cell.
     * If you wish to check whether the proximity sense the Power Cell, use {@link #isBallClose()} instead.
     *
     * @return whether the {@link #proximity} lost the Power Cell.
     */
    private boolean isBallAway() {
        return getRaw() < minDistance;
    }
}
