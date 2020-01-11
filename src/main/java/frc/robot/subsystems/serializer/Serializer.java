package frc.robot.subsystems.serializer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private TalonSRX exitMotor = new TalonSRX(EXIT_MOTOR);
    private VictorSPX entryMotor = new VictorSPX(ENTRY_MOTOR);
    private AnalogInput entryProximity = new AnalogInput(ENTRY_PROXIMITY);
    private AnalogInput integrationProximity = new AnalogInput(MIDDLE_PROXIMITY);
    private AnalogInput exitProximity = new AnalogInput(EXIT_PROXIMITY);
    private int ballsCount = 3;
    private double startLocation, endLocation;
    private boolean movingUp;
    private boolean entryBallInside, integrationBallInside;

    public Serializer() {
        exitMotor.configFactoryDefault();
        entryMotor.configFactoryDefault();

        exitMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, TALON_TIMEOUT_MS);
        exitMotor.setSensorPhase(EXIT_PHASED);
        entryMotor.setSensorPhase(ENTRY_PHASED);
        exitMotor.setInverted(EXIT_INVERTED);
        entryMotor.setInverted(ENTRY_INVERTED);

        exitMotor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT_MS);
        exitMotor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT_MS);
        exitMotor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT_MS);

        exitMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        exitMotor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT_MS);
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
        if (isEntryProximityPressed() && movingUp) {
            incrementBallsCount(1);
            startLocation = getEncoderPosition();
        }

        if (isExitProximityReleased() && movingUp) {
            decrementBallsCount(1);
            endLocation = getEncoderPosition();
        }

        if (isEntryProximityReleased() && !movingUp && entryBallInside) {
            decrementBallsCount(1);
            startLocation = getEncoderPosition();
        }
        entryBallInside = isEntryProximityPressed();
    }

    /**
     * retrieve the current {@link #entryMotor}'s velocity.
     *
     * @return the velocity of the {@link #entryMotor}.
     */
    public int getEntryVelocity() {
        return entryMotor.getSelectedSensorVelocity();
    }

    /**
     * set the velocity for the {@link #entryMotor}.
     *
     * @param meterPerSecond the speed to apply on {@link #entryMotor}.
     */
    public void setEntryVelocity(double meterPerSecond) {
        movingUp = (meterPerSecond >= 0);
        entryMotor.set(ControlMode.Velocity, model.toUnits(meterPerSecond) / 10);
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
     * @param location the relative location you want the {@link #exitMotor} to move.
     */
    public void setExitVelocity(double location) {
        movingUp = (location >= 0);
        exitMotor.set(ControlMode.MotionMagic, location);
    }

    /**
     * retrieve whether the {@link #entryProximity} is pressed by a ball.
     *
     * @return whether the {@link #entryProximity} is pressed.
     */
    public boolean isEntryProximityPressed() {
        return entryProximity.getVoltage() > ENTRY_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #entryProximity} is released by a ball.
     *
     * @return whether the {@link #entryProximity} is released.
     */
    public boolean isEntryProximityReleased() {
        return entryProximity.getVoltage() < ENTRY_PROXIMITY_MIN_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #integrationProximity} is pressed by a ball.
     *
     * @return whether the {@link #integrationProximity} is pressed.
     */
    public boolean isIntegrationProximityPressed() {
        return integrationProximity.getVoltage() > INTEGRATION_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #integrationProximity} is released by a ball.
     *
     * @return whether the {@link #integrationProximity} is released.
     */
    public boolean isIntegrationProximityReleased() {
        return integrationProximity.getVoltage() < INTEGRATION_PROXIMITY_MIN_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #exitProximity} is pressed by a ball.
     *
     * @return whether the {@link #exitProximity} is pressed.
     */
    public boolean isExitProximityPressed() {
        return exitProximity.getVoltage() > EXIT_PROXIMITY_MAX_VOLTAGE;
    }

    /**
     * retrieve whether the {@link #exitProximity} is released by a ball.
     *
     * @return whether the {@link #exitProximity} is released.
     */
    public boolean isExitProximityReleased() {
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
        setExitVelocity(getEncoderPosition() + location);
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
     * move the the first ball to the {@link #exitProximity}.
     * note that the other balls still move until the first ball will reach to the {@link #exitProximity}.
     */
    public void maximizeConveyor() {
        moveConveyor(endLocation); //TODO choose real number
    }

    /**
     * move the the last ball to the {@link #entryProximity}.
     * note that the other balls still move until the last ball will reach to the {@link #entryProximity}.
     */
    public void minimizeConveyor() {
        moveConveyor(startLocation, -0.1); //TODO choose real number
    }

    /**
     * retrieve the balls count that the proximities noticed.
     *
     * @return the balls count that the proximities noticed.
     */
    public int getBallsCount() {
        return ballsCount;
    }

    /**
     * change the amount of balls in the conveyor.
     * notice that this method will only change the variable {@link #ballsCount},
     * if you wish to move the conveyor, use {@link #feed()} or {@link #drop()} instead.
     *
     * @param ballsCount the amount of ball.
     */
    private void setBallsCount(int ballsCount) {
        this.ballsCount = Math.max(0, Math.min(ballsCount, MAX_BALLS_COUNT));
    }

    /**
     * increment the amount of {@link #ballsCount}.
     *
     * @param by the number of balls you want to increment.
     */
    private void incrementBallsCount(int by) {
        setBallsCount(ballsCount + by);
    }

    /**
     * decrement the amount of {@link #ballsCount}.
     *
     * @param by the number of balls you want to decrement.
     */
    private void decrementBallsCount(int by) {
        setBallsCount(ballsCount - by);
    }

    //TODO choose reasonable value

    /**
     * feed the conveyor in one ball per run.
     */
    public void feed() {
        exitMotor.set(ControlMode.PercentOutput, 60);
        entryMotor.set(ControlMode.PercentOutput, 70);
    }

    /**
     * drop from the conveyor one ball per run.
     */
    public void drop() {
        if (!isEntryProximityReleased())
            exitMotor.set(ControlMode.PercentOutput, -10);
        entryMotor.set(ControlMode.PercentOutput, -70);
    }

    /**
     * stop the conveyor's motors from moving.
     */
    public void stop() {
        exitMotor.set(ControlMode.PercentOutput, 0);
        entryMotor.set(ControlMode.PercentOutput, 0);
    }
}