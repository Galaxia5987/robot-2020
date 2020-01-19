package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.DeadbandProximity;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.TALON_TIMEOUT_MS;
import static frc.robot.Ports.Conveyor.*;

/**
 * @author Barel
 * @version 1.0.0
 * <p>
 * set the default methods for the commands.
 * {@using TalonSRX}
 * {@using VictorSPX}
 * {@using 3xProximities}
 */
public class Conveyor extends SubsystemBase {
    UnitModel unitsConverter = new UnitModel(TICK_PER_METERS);
    private TalonSRX motor = new TalonSRX(MOTOR);
    private DeadbandProximity feederProximity = new DeadbandProximity(FEEDER_PROXIMITY, FEEDER_PROXIMITY_MIN_VOLTAGE, FEEDER_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity conveyorProximity = new DeadbandProximity(CONVEYOR_PROXIMITY, CONVEYOR_PROXIMITY_MIN_VOLTAGE, CONVEYOR_PROXIMITY_MAX_VOLTAGE);
    private int ballsCount = 3;
    private double startLocation, endLocation;

    public Conveyor() {
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, TALON_TIMEOUT_MS);
        motor.setSensorPhase(SENSOR_INVERTED);
        motor.setInverted(MOTOR_INVERTED);

        motor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT_MS);
        motor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT_MS);
        motor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT_MS);

        motor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        motor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT_MS);
        motor.configPeakCurrentLimit(MAX_CURRENT);
        motor.configClosedloopRamp(RAMP_RATE);

        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        updateSensors();

        if (feederProximity.getState()) {
            if (feederProximity.getToggle())
                incrementBallsCount(1);
            startLocation = getConveyorPosition();
        }

        if (conveyorProximity.getState()) {
            if (conveyorProximity.getToggle() && (getConveyorVelocity() > 0))
                decrementBallsCount(1);
            endLocation = getConveyorPosition();
        }
    }

    private void updateSensors() {
        feederProximity.update();
        conveyorProximity.update();
    }

    /**
     * retrieve the current {@link #motor}'s encoder position.
     *
     * @return the current {@link #motor}'s encoder position.
     */
    public double getConveyorPosition() {
        return unitsConverter.toTicks(motor.getSelectedSensorPosition());
    }

    /**
     * set the relative location for the {@link #motor}.
     *
     * @param location the relative location you want the conveyor to move.
     */
    private void setConveyorPosition(double location) {
        motor.set(ControlMode.MotionMagic, location);
    }

    /**
     * retrieve the current {@link #motor}'s velocity.
     *
     * @return the velocity of the {@link #motor}.
     */
    public int getConveyorVelocity() {
        return motor.getSelectedSensorVelocity() * 10 * TICK_PER_METERS; //TODO change to unitModel once unitmodel is fixed.
    }

    /**
     * move the conveyor to the desired relative location.
     *
     * @param location the desired relative location.
     */
    public void moveConveyor(double location) {
        setConveyorPosition(getConveyorPosition() + location);
    }

    /**
     * move the the last Power Cell to the lower end of the conveyor.
     * note that the other Power Cells still move until the last Power Cell will reach to the {@link #feederProximity}.
     */
    public void minimizeConveyor() {
        moveConveyor(startLocation); //TODO choose real number
    }

    /**
     * move the first Power Cell to the higher end of the conveyor.
     * note that the other Power Cells still move until the first Power Cell will reach to the {@link #conveyorProximity}.
     */
    public void maximizeConveyor() {
        moveConveyor(endLocation); //TODO choose real number
    }

    /**
     * feed the conveyor in one Power Cell per run.
     */
    public void feed() {
        motor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_FEED_VELOCITY);
    }

    /**
     * stop the conveyor's motors from moving.
     */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * increment the amount of {@link #ballsCount}.
     *
     * @param amount the number of Power Cells you want to increment.
     */
    private void incrementBallsCount(int amount) {
        setBallsCount(ballsCount + amount);
    }

    /**
     * decrement the amount of {@link #ballsCount}.
     *
     * @param amount the number of Power Cells you want to decrement.
     */
    private void decrementBallsCount(int amount) {
        setBallsCount(ballsCount - amount);
    }

    /**
     * retrieve the Power Cells count that the proximities noticed.
     *
     * @return the Power Cells count that the proximities noticed.
     */
    public int getBallsCount() {
        return ballsCount;
    }

    /**
     * change the amount of Power Cells in the conveyor.
     * notice that this method will only change the variable {@link #ballsCount},
     * if you wish to move the conveyor, use {@link #feed()} instead.
     *
     * @param ballsCount the amount of Power Cell.
     */
    private void setBallsCount(int ballsCount) {
        this.ballsCount = ballsCount;
    }

    public boolean feederSensedObject() {
        return feederProximity.getState();
    }
}