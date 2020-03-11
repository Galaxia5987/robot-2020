package frc.robot.utilities;

import edu.wpi.first.wpilibj.AnalogInput;

import java.util.function.Supplier;

/**
 * This class is a modification of the regular proximity class, which adds a deadband, to insure that the sensor
 * does not toggle multiple times.
 * this is crucial for the conveyor, because the amount of balls that enter are counted by the amount of times
 * the sensor switches from false to true.
 */
public class DeadbandProximity {
    private double minVoltage;
    private double maxVoltage;
    private Supplier<Integer> value;
    private boolean objectSensed = false;
    private boolean toggle = false;

    public DeadbandProximity(Supplier<Integer> value, double minVoltage, double maxVoltage) {
        this.value = value;
        this.maxVoltage = maxVoltage;
        this.minVoltage = minVoltage;
    }

    /**
     * Check whether the objectSensed was sensed by the proximity.
     * An object is sensed by the proximity sensor once the proximity value goes above maxVoltage
     * It will only say that it does not see a target anymore, when the value goes under minVoltage
     * this is to ensure that the proximity doesn't toggle rapidly because of sensor noise.
     *
     * Side note: for the toggle to work correctly, the update needs to be called only once per robot loop. (sorry Dan!)
     */
    public void update() {
        boolean lastState = objectSensed;
        if (isObjectAway()) {
            objectSensed = false;
        } else if (isObjectClose()) {
            objectSensed = true;
        }
        toggle = objectSensed != lastState;
    }

    /**
     * Return whether the proximity is sensing an object.
     *
     * @return whether the object was sensed by the proximity.
     */
    public boolean getState() {
        return objectSensed;
    }

    /**
     * Return whether the proximity sensed and then lost the object
     *
     * @return whether the object passed the proximity sensor
     */
    public boolean getToggle(){
        return toggle;
    }

    public int getValue(){
        return value.get();
    }
    /**
     * Return whether the proximity senses an object.
     * If you wish to check whether the proximity lost the object, use {@link #isObjectAway()} instead.
     *
     * @return whether the proximity sense a object.
     */
    private boolean isObjectClose() {
        return value.get() > maxVoltage;
    }

    public void resetToggle(){
        toggle = false;
    }

    /**
     * Return whether the proximity lost the object.
     * If you wish to check whether the proximity sense the object, use {@link #isObjectClose()} instead.
     *
     * @return whether the proximity lost the object.
     */
    private boolean isObjectAway() {
        return value.get() < minVoltage;
    }
}
