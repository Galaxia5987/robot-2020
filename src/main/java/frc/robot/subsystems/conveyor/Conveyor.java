package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
    private VictorSPX feederMotor = new VictorSPX(FEEDER_MOTOR);
    private TalonSRX conveyorMotor = new TalonSRX(CONVEYOR_MOTOR);
    private DeadbandProximity feederProximity = new DeadbandProximity(FEEDER_PROXIMITY, FEEDER_PROXIMITY_MIN_VOLTAGE, FEEDER_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity integrationProximity = new DeadbandProximity(INTEGRATION_PROXIMITY, INTEGRATION_PROXIMITY_MIN_VOLTAGE, INTEGRATION_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity conveyorProximity = new DeadbandProximity(CONVEYOR_PROXIMITY, CONVEYOR_PROXIMITY_MIN_VOLTAGE, CONVEYOR_PROXIMITY_MAX_VOLTAGE);
    private int ballsCount = 3;
    private double startLocation, endLocation;

    public Conveyor() {
        conveyorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, TALON_TIMEOUT_MS);
        conveyorMotor.setSensorPhase(CONVEYOR_SENSOR_INVERTED);
        conveyorMotor.setInverted(CONVEYOR_MOTOR_INVERTED);
        feederMotor.setInverted(FEEDER_MOTOR_INVERTED);

        conveyorMotor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT_MS);
        conveyorMotor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT_MS);
        conveyorMotor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT_MS);

        conveyorMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        conveyorMotor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT_MS);
        conveyorMotor.configPeakCurrentLimit(MAX_CURRENT);
        conveyorMotor.configClosedloopRamp(RAMP_RATE);

        conveyorMotor.configVoltageCompSaturation(12);
        feederMotor.configVoltageCompSaturation(12);
        conveyorMotor.enableVoltageCompensation(true);
        feederMotor.enableVoltageCompensation(true);
        conveyorMotor.setSelectedSensorPosition(0);
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
        integrationProximity.update();
        conveyorProximity.update();
    }

    /**
     * retrieve the current {@link #conveyorMotor}'s encoder position.
     *
     * @return the current {@link #conveyorMotor}'s encoder position.
     */
    public double getConveyorPosition() {
        return unitsConverter.toTicks(conveyorMotor.getSelectedSensorPosition());
    }

    /**
     * set the speed for the {@link #feederMotor}.
     *
     * @param speed the speed to apply on {@link #feederMotor}.
     *                 be noted you should enter a value between -1 to 1.
     */
    public void setFeederSpeed(double speed) {
        feederMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * retrieve the current {@link #conveyorMotor}'s velocity.
     *
     * @return the velocity of the {@link #conveyorMotor}.
     */
    public int getConveyorVelocity() {
        return conveyorMotor.getSelectedSensorVelocity();
    }

    /**
     * set the relative location for the {@link #conveyorMotor}.
     *
     * @param location the relative location you want the conveyor to move.
     */
    public void setConveyorPosition(double location) {
        conveyorMotor.set(ControlMode.MotionMagic, location);
    }

    /**
     * move the conveyor to the desired relative location.
     * if you wish to use the default, use {@link #moveConveyor(double)} instead.
     *
     * @param location        the desired relative location.
     * @param metersPerSecond the metersPerSecond you want the motor to move.
     */
    public void moveConveyor(double location, double metersPerSecond) {
        setConveyorPosition(getConveyorPosition() + location);
        setFeederSpeed(metersPerSecond);
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
     * note that the other Power Cells still move until the last Power Cell will reach to the {@link #feederProximity}.
     */
    public void minimizeConveyor() {
        moveConveyor(startLocation, -0.1); //TODO choose real number
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
        conveyorMotor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_FEED_VELOCITY);
        feederMotor.set(ControlMode.PercentOutput, FEEDER_MOTOR_FEED_VELOCITY);
    }

    /**
     * stop the conveyor's motors from moving.
     */
    public void stop() {
        conveyorMotor.set(ControlMode.PercentOutput, 0);
        feederMotor.set(ControlMode.PercentOutput, 0);
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
     * change the amount of Power Cells in the conveyor.
     * notice that this method will only change the variable {@link #ballsCount},
     * if you wish to move the conveyor, use {@link #feed()} instead.
     *
     * @param ballsCount the amount of Power Cell.
     */
    private void setBallsCount(int ballsCount) {
        ballsCount = Math.min(ballsCount, MAX_BALLS_AMOUNT);
    }

    /**
     * retrieve the Power Cells count that the proximities noticed.
     *
     * @return the Power Cells count that the proximities noticed.
     */
    public int getBallsCount() {
        return ballsCount;
    }

    public boolean feederSensedObject() {
        return feederProximity.getState();
    }
}