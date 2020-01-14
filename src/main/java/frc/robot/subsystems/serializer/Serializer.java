package frc.robot.subsystems.serializer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Serializer.*;
import static frc.robot.Ports.Serializer.*;

/**
 * @author Barel
 * @version 1.0.0
 * <p>
 * set the default methods for the commands.
 * {@using TalonSRX}
 * {@using VictorSPX}
 * {@using 3xProximities}
 */
public class Serializer extends SubsystemBase {
    UnitModel model = new UnitModel(TICK_PER_METERS);
    private VictorSPX entryMotor = new VictorSPX(ENTRY_MOTOR);
    private TalonSRX exitMotor = new TalonSRX(EXIT_MOTOR);
    private AnalogInput entryProximity = new AnalogInput(ENTRY_PROXIMITY);
    private AnalogInput integrationProximity = new AnalogInput(INTEGRATION_PROXIMITY);
    private AnalogInput exitProximity = new AnalogInput(EXIT_PROXIMITY);
    private int ballsCount = 3;
    private double startLocation, endLocation;
    private boolean ballInEntryPosition, ballInIntegrationPosition, ballInExitPosition;
    private boolean movingUp;

    public Serializer() {
        exitMotor.configFactoryDefault();
        entryMotor.configFactoryDefault();

        exitMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, Constants.TALON_TIMEOUT_MS);
        exitMotor.setSensorPhase(EXIT_SENSOR_PHASED);
        exitMotor.setInverted(EXIT_INVERTED);
        entryMotor.setInverted(ENTRY_INVERTED);

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
        if (isBallSensedInEntry() && isBallsMovingUp() && !ballInEntryPosition) {
            incrementBallsCount(1);
            startLocation = getEncoderPosition();
        }

        if (isBallLostInEntry() && !isBallsMovingUp() && !ballInEntryPosition) {
            decrementBallsCount(1);
            startLocation = getEncoderPosition();
        }

        if (isBallLostInExit() && isBallsMovingUp() && !ballInExitPosition) {
            decrementBallsCount(1);
            endLocation = getEncoderPosition();
        }

        ballInExitPosition = !isBallLostInExit();
        ballInEntryPosition = !isBallLostInEntry();
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
     * retrieve whether the {@link #entryProximity} sense a Power Cell.
     * If you wish to check whether the proximity lost the Power Cell, use {@link #isBallLostInEntry()} instead.
     *
     * @return whether the {@link #entryProximity} sense a Power Cell.
     */
    public boolean isBallSensedInEntry() {
        return entryProximity.getVoltage() > ENTRY_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #entryProximity} lost the Power Cell.
     * If you wish to check whether the proximity sensed a Power Cell, use {@link #isBallSensedInIntegration()} instead.
     *
     * @return whether the {@link #entryProximity} lost the Power Cell.
     */
    public boolean isBallLostInEntry() {
        return entryProximity.getVoltage() < ENTRY_PROXIMITY_MIN_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #integrationProximity} sense a Power Cell.
     * If you wish to check whether the proximity lost the Power Cell, use {@link #isBallLostInIntegration()} instead.
     *
     * @return whether the {@link #integrationProximity} sense a Power Cell.
     */
    public boolean isBallSensedInIntegration() {
        return integrationProximity.getVoltage() > INTEGRATION_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #integrationProximity} lost the Power Cell.
     * If you wish to check whether the proximity sensed a Power Cell, use {@link #isBallSensedInExit()} instead.
     *
     * @return whether the {@link #integrationProximity} lost the Power Cell.
     */
    public boolean isBallLostInIntegration() {
        return integrationProximity.getVoltage() < INTEGRATION_PROXIMITY_MIN_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #exitProximity} sense a Power Cell.
     * If you wish to check whether the proximity lost the Power Cell, use {@link #isBallLostInExit()} instead.
     *
     * @return whether the {@link #exitProximity} sense a Power Cell.
     */
    public boolean isBallSensedInExit() {
        return exitProximity.getVoltage() > EXIT_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #exitProximity} lost the Power Cell.
     *
     * @return whether the {@link #exitProximity} lost the Power Cell.
     */
    public boolean isBallLostInExit() {
        return exitProximity.getVoltage() < EXIT_PROXIMITY_MIN_VOLTAGE;
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
     * retrieve the current {@link #exitMotor}'s encoder position.
     *
     * @return the current {@link #exitMotor}'s encoder position.
     */
    public double getEncoderPosition() {
        return model.toTicks(exitMotor.getSelectedSensorPosition());
    }

    /**
     * move the first Power Cell to the higher end of the conveyor.
     * note that the other Power Cells still move until the first Power Cell will reach to the {@link #exitProximity}.
     */
    public void maximizeConveyor() {
        moveConveyor(endLocation); //TODO choose real number
    }

    /**
     * move the the last Power Cell to the lower end of the conveyor.
     * note that the other Power Cells still move until the last Power Cell will reach to the {@link #entryProximity}.
     */
    public void minimizeConveyor() {
        moveConveyor(startLocation, -0.1); //TODO choose real number
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
        this.ballsCount = Math.max(0, Math.min(ballsCount, MAX_BALLS_COUNT));
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

    public boolean isBallsMovingUp() {
        return movingUp;
    }
}