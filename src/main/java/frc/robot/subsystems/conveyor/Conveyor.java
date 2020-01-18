package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.DeadbandProximity;

import static frc.robot.Constants.Conveyor.*;
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
    private VictorSPX entryMotor = new VictorSPX(ENTRY_MOTOR);
    private TalonSRX exitMotor = new TalonSRX(EXIT_MOTOR);
    private DeadbandProximity entryProximity = new DeadbandProximity(ENTRY_PROXIMITY, ENTRY_PROXIMITY_MIN_VOLTAGE, ENTRY_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity integrationProximity = new DeadbandProximity(INTEGRATION_PROXIMITY, INTEGRATION_PROXIMITY_MIN_VOLTAGE, ENTRY_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity exitProximity = new DeadbandProximity(EXIT_PROXIMITY, EXIT_PROXIMITY_MIN_VOLTAGE, EXIT_PROXIMITY_MAX_VOLTAGE);
    private int ballsCount = 3;
    private double startLocation, endLocation;
    private boolean movingUp;

    public Conveyor() {
        exitMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, Constants.TALON_TIMEOUT_MS);
        exitMotor.setSensorPhase(EXIT_SENSOR_INVERTED);
        exitMotor.setInverted(EXIT_MOTOR_INVERTED);
        entryMotor.setInverted(ENTRY_MOTOR_INVERTED);

        exitMotor.config_kP(TALON_PID_SLOT, KP, Constants.TALON_TIMEOUT_MS);
        exitMotor.config_kI(TALON_PID_SLOT, KI, Constants.TALON_TIMEOUT_MS);
        exitMotor.config_kD(TALON_PID_SLOT, KD, Constants.TALON_TIMEOUT_MS);

        exitMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        exitMotor.configMotionAcceleration(CRUISE_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        exitMotor.configPeakCurrentLimit(MAX_CURRENT);
        exitMotor.configClosedloopRamp(RAMP_RATE);

        exitMotor.configVoltageCompSaturation(12);
        entryMotor.configVoltageCompSaturation(12);
        exitMotor.enableVoltageCompensation(true);
        entryMotor.enableVoltageCompensation(true);
        exitMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        updateSensors();
        
        if (entryProximity.getState()) {
            if (entryProximity.getToggle())
                incrementBallsCount(1);
            startLocation = getEncoderPosition();
        }
        if (exitProximity.getState()) {
            if (exitProximity.getToggle() && (getExitVelocity() > 0))
                decrementBallsCount(1);
            endLocation = getEncoderPosition();
        }
    }

    private void updateSensors(){
        entryProximity.update();
        integrationProximity.update();
        exitProximity.update();
    }

    /**
     * retrieve the current {@link #exitMotor}'s encoder position.
     *
     * @return the current {@link #exitMotor}'s encoder position.
     */
    public double getEncoderPosition() {
        return unitsConverter.toTicks(exitMotor.getSelectedSensorPosition());
    }

    /**
     * return whether the balls are moving up.
     *
     * @return whether the balls are moving up.
     */
    public boolean isBallsMovingUp() {
        return movingUp;
    }

    /**
     * set the velocity for the {@link #entryMotor}.
     *
     * @param velocity the speed to apply on {@link #entryMotor}.
     *                 be noted you should enter a value between -1 to 1.
     */
    public void setEntryVelocity(double velocity) {
        movingUp = (velocity >= 0);
        entryMotor.set(ControlMode.PercentOutput, velocity);
    }

    /**
     * retrieve the current {@link #exitMotor}'s velocity.
     *
     * @return the velocity of the {@link #exitMotor}.
     */
    public int getExitVelocity() {
        return exitMotor.getSelectedSensorVelocity();
    }

    /**
     * set the relative location for the {@link #exitMotor}.
     *
     * @param location the relative location you want the conveyor to move.
     */
    public void setLocationToExitMotor(double location) {
        movingUp = (location >= 0);
        exitMotor.set(ControlMode.MotionMagic, location);
    }

    /**
     * move the conveyor to the desired relative location.
     * if you wish to use the default, use {@link #moveConveyor(double)} instead.
     *
     * @param location        the desired relative location.
     * @param metersPerSecond the metersPerSecond you want the motor to move.
     */
    public void moveConveyor(double location, double metersPerSecond) {
        setLocationToExitMotor(getEncoderPosition() + location);
        setEntryVelocity(metersPerSecond);
    }

    /**
     * the default moveConveyor method, it move the conveyor to the desired relative location.
     *
     * @param location the desired relative location.
     */
    public void moveConveyor(double location) {
        moveConveyor(location, 0.5); //TODO Change the speed to reasonable value
    }

    /**
     * move the the last Power Cell to the lower end of the conveyor.
     * note that the other Power Cells still move until the last Power Cell will reach to the {@link #entryProximity}.
     */
    public void minimizeConveyor() {
        moveConveyor(startLocation, -0.1); //TODO choose real number
    }

    /**
     * move the first Power Cell to the higher end of the conveyor.
     * note that the other Power Cells still move until the first Power Cell will reach to the {@link #exitProximity}.
     */
    public void maximizeConveyor() {
        moveConveyor(endLocation); //TODO choose real number
    }

    /**
     * retrieve the Power Cells count that the proximities noticed.
     *
     * @return the Power Cells count that the proximities noticed.
     */
    public int getBallsCount() {
        return ballsCount;
    }

    //TODO choose reasonable value
    /**
     * feed the conveyor in one Power Cell per run.
     */
    public void feed() {
        exitMotor.set(ControlMode.PercentOutput, EXIT_MOTOR_FEED_VELOCITY);
        entryMotor.set(ControlMode.PercentOutput, ENTRY_MOTOR_FEED_VELOCITY);
    }

    /**
     * stop the conveyor's motors from moving.
     */
    public void stop() {
        exitMotor.set(ControlMode.PercentOutput, 0);
        entryMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * change the amount of Power Cells in the conveyor.
     * notice that this method will only change the variable {@link #ballsCount},
     * if you wish to move the conveyor, use {@link #feed()} instead.
     *
     * @param ballsCount the amount of Power Cell.
     */
    private void setBallsCount(int ballsCount) {
        this.ballsCount = Math.min(ballsCount, MAX_BALLS_AMOUNT);
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
}