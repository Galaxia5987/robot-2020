package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.DeadbandProximity;
import frc.robot.utilities.State;

import static frc.robot.Constants.Conveyor.*;
import static frc.robot.Constants.TALON_TIMEOUT;
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
    private DeadbandProximity intakeProximity = new DeadbandProximity(INTAKE_PROXIMITY, INTAKE_PROXIMITY_MIN_VOLTAGE, INTAKE_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity conveyorProximity = new DeadbandProximity(CONVEYOR_PROXIMITY, CONVEYOR_PROXIMITY_MIN_VOLTAGE, CONVEYOR_PROXIMITY_MAX_VOLTAGE);
    private Solenoid gate = new Solenoid(GATE); //mechanical stop
    private int ballsCount = 3;

    public Conveyor() {
        motor.setInverted(MOTOR_INVERTED);

        motor.config_kP(0, KP, TALON_TIMEOUT);
        motor.config_kI(0, KI, TALON_TIMEOUT);
        motor.config_kD(0, KD, TALON_TIMEOUT);

        motor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        motor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT);
        motor.configPeakCurrentLimit(MAX_CURRENT);
        motor.configClosedloopRamp(RAMP_RATE);
    }

    @Override
    public void periodic() {
        updateSensors();
        //If the intake senses an object, and it hasn't in the previous state, and the wheels are turning outwards, add a ball to the count
        if (intakeProximity.getState() && intakeProximity.getToggle() && (getConveyorSpeed() > 0))
                incrementBallsCount(1);
        //If the conveyor proximity loses an object, and it hasn't been off before and the conveyor is spinning outwards, remove a ball from the count
        if (!conveyorProximity.getState() && conveyorProximity.getToggle() && (getConveyorSpeed() > 0))
                decrementBallsCount(1);
    }

    private void updateSensors() {
        intakeProximity.update();
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
     * set the speed for the {@link #motor}.
     *
     * @param speed the speed you want the conveyor to move.
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
        if(isGateOpen()) return;
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

    /**
     * Return if the intake is seeing an object
     * @return true if a power cell is in front of the sensor.
     */
    public boolean intakeSensedObject() {
        return intakeProximity.getState();
    }

    public boolean isGateOpen(){
        return gate.get();
    }

    public void openGate(boolean open){
        gate.set(open);
    }

    /**
     * Alternative way to open the gate, if booleans are too confusing to work with.
     * OPEN is in the state where the stopper is open, and balls can feed freely.
     * TOGGLE state is ignored.
     *
     * @param state state of the stopper, OPEN / CLOSE
     */
    public void openGate(State state){
        switch (state){
            case OPEN:
                openGate(true);
                break;
            case CLOSE:
                openGate(false);
                break;
            case TOGGLE:
                break;
        }
    }
}