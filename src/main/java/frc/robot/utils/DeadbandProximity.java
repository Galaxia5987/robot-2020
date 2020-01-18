package frc.robot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class is a modification of the regular proximity class, which adds a deadband, to insure that the sensor
 * does not toggle multiple times.
 * this is crucial for the conveyer, because the amount of balls that enter are counted by the amount of times
 * the sensor switches from false to true.
 */
public class DeadbandProximity {
    private AnalogInput proximity;
    private double minVoltage;
    private double maxVoltage;
    private boolean objectSensed = false;

    public DeadbandProximity(int port, double minVoltage, double maxVoltage) {
        proximity = new AnalogInput(port);
        this.maxVoltage = maxVoltage;
        this.minVoltage = minVoltage;
    }

    /**
     * check whether the objectSensed was sensed by the proximity.
     * An object is sensed by the proximity sensor once the proximity value goes above maxVoltage
     * It will only say that it does not see a target anymore, when the value goes under minVoltage
     * this is to ensure that the proximity doesn't toggle rapidly because of sensor noise.
     */
    public void update() {
        if (isObjectAway()) {
            objectSensed = false;
        } else if (isObjectClose()) {
            objectSensed = true;
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
     * Retrieves whether the proximity is sensing an object.
     *
     * @return whether the ball was sensed by the proximity.
     */
    public boolean isObjectSensed() {
        return objectSensed;
    }

    /**
     * retrieve whether the {@link #proximity} sense a Power Cell.
     * If you wish to check whether the proximity lost the Power Cell, use {@link #isObjectAway()} ()} instead.
     *
     * @return whether the {@link #proximity} sense a Power Cell.
     */
    private boolean isObjectClose() {
        return getRaw() > maxVoltage;
    }

    /**
     * retrieve whether the {@link #proximity} lost the Power Cell.
     * If you wish to check whether the proximity sense the Power Cell, use {@link #isObjectClose()} instead.
     *
     * @return whether the {@link #proximity} lost the Power Cell.
     */
    private boolean isObjectAway() {
        return getRaw() < minVoltage;
    }
}
