package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
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
    private Solenoid solenoid = new Solenoid(SOLENOID);
    private int ballsCount = 3;

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
            if (feederProximity.getToggle() && (getConveyorSpeed() > 0))
                incrementBallsCount(1);
        }

        if (conveyorProximity.getState()) {
            if (conveyorProximity.getToggle() && (getConveyorSpeed() > 0))
                decrementBallsCount(1);
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
     * set the relative speed for the {@link #motor}.
     *
     * @param speed the relative speed you want the conveyor to move.
     */
    public void setConveyorSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * retrieve the current {@link #motor}'s velocity.
     *
     * @return the velocity of the {@link #motor}.
     */
    public double getConveyorSpeed() {
        return motor.getMotorOutputPercent(); //TODO change to unitModel once unitmodel is fixed.
    }

    /**
     * feed the conveyor in one Power Cell per run.
     */
    public void feed() {
        if(solenoid.get()) return;//TODO Check
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
        setBallsCount(getBallsCount() + amount);
    }

    /**
     * decrement the amount of {@link #ballsCount}.
     *
     * @param amount the number of Power Cells you want to decrement.
     */
    private void decrementBallsCount(int amount) {
        setBallsCount(getBallsCount() - amount);
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