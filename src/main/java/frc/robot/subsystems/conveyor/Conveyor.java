package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utilities.CustomDashboard;
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
    private UnitModel unitConverter = new UnitModel(TICK_PER_METERS);
    private TalonSRX motor = new TalonSRX(MOTOR);
    private VictorSPX funnel = new VictorSPX(FUNNEL);
    
    
    private DeadbandProximity shooterProximity = new DeadbandProximity(new AnalogInput(SHOOTER_PROXIMITY)::getValue, SHOOTER_PROXIMITY_MIN_VOLTAGE, SHOOTER_PROXIMITY_MAX_VOLTAGE);
    private DeadbandProximity intakeProximity;
    private DoubleSolenoid gateA = null; //mechanical stop
    private Solenoid gateB = null; //mechanical stop
    private int ballsCount = STARTING_AMOUNT;

    public Conveyor(Intake intake) {
        intakeProximity = new DeadbandProximity(intake::getSensorValue, INTAKE_PROXIMITY_MIN_VOLTAGE, INTAKE_PROXIMITY_MAX_VOLTAGE);

        motor.configFactoryDefault();
        funnel.configFactoryDefault();

        motor.setInverted(MOTOR_INVERTED);
        funnel.setInverted(FUNNEL_INVERTED);

        motor.config_kP(0, KP, TALON_TIMEOUT);
        motor.config_kI(0, KI, TALON_TIMEOUT);
        motor.config_kD(0, KD, TALON_TIMEOUT);

        motor.configMotionCruiseVelocity(unitConverter.toTicks100ms(CRUISE_VELOCITY));
        motor.configMotionAcceleration(unitConverter.toTicks100ms(CRUISE_ACCELERATION));
        motor.configPeakCurrentLimit(0);
        motor.configContinuousCurrentLimit(30);
        motor.enableCurrentLimit(true);
        motor.configClosedloopRamp(RAMP_RATE);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12.0);

        funnel.enableVoltageCompensation(true);
        funnel.configVoltageCompSaturation(12.0);

        if (Robot.isRobotA)
            gateA = new DoubleSolenoid(FORWARD_GATE, REVERSE_GATE);
        else
            gateB = new Solenoid(GATE);
    }

    @Override
    public void periodic() {
        updateSensors();
        //If the intake senses an object, and it hasn't in the previous state, and the wheels are turning outwards, add a ball to the count
        if (intakeProximity.getToggle() && intakeSensedBall() && (getPower() >= 0)) {
            incrementBallsCount(1);
            intakeProximity.resetToggle();
            shooterProximity.resetToggle();
        }
        //If the conveyor proximity loses an object, and it hasn't been off before and the conveyor is spinning outwards, remove a ball from the count
        //Additionally, if the conveyor outtakes a ball and the sensor sees the ball pass it, decrement the count aswell.
        if ((!shooterProximity.getState() && shooterProximity.getToggle() && (getPower() > 0)) ||
                (intakeProximity.getToggle() && !intakeSensedBall() && (getPower() < 0))) {
            decrementBallsCount(1);
            intakeProximity.resetToggle();
            shooterProximity.resetToggle();
        }
        CustomDashboard.setBallCount(getBallsCount());
        CustomDashboard.setGate(isGateOpen());
    }

    private void updateSensors() {
        intakeProximity.update();
        shooterProximity.update();
    }

    /**
     * retrieve the current motor's encoder position.
     *
     * @return the current motor's encoder position.
     */
    public double getPosition() {
        return unitConverter.toUnits(motor.getSelectedSensorPosition());
    }

    /**
     * return the power which the motor is turning at.
     *
     * @return the power of the motor.
     */
    public double getPower() {
        return motor.getMotorOutputPercent();
    }

    /**
     * set the power for the {@link #motor}.
     *
     * @param power The power given to the motor from -1 to 1.
     */
    public void setConveyorPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void setFunnelPower(double power){
        funnel.set(ControlMode.PercentOutput, power);
    }

    /**
     * feed the conveyor in one Power Cell per run.
     */
    public void feed() {
        if (!isGateOpen()) return;
        motor.set(ControlMode.PercentOutput, CONVEYOR_MOTOR_FEED_POWER);
    }

    /**
     * stop the conveyor's motors from moving.
     */
    @Deprecated
    public void stop() {
        stopAll();
    }

    public void stopAll() {
        stopConveyor();
        stopFunnel();
    }

    public void stopConveyor() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void stopFunnel() {
        funnel.set(ControlMode.PercentOutput, 0);
    }

    /**
     * increment the amount of {@link #ballsCount}.
     *
     * @param amount the number of Power Cells you want to increment.
     */
    public void incrementBallsCount(int amount) {
        setBallsCount(getBallsCount() + amount);
    }

    /**
     * decrement the amount of {@link #ballsCount}.
     *
     * @param amount the number of Power Cells you want to decrement.
     */
    public void decrementBallsCount(int amount) {
        setBallsCount(getBallsCount() - amount);
    }

    /**
     * @return the amount of Power Cells on the robot, counted by the proximity sensors
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
    public void setBallsCount(int ballsCount) {
        this.ballsCount = ballsCount;
    }

    /**
     * @return whether a power cell is beneath the stopper.
     */
    public boolean shooterSensedBall() {
        return shooterProximity.getState();
    }

    /**
     * @return whether a power cell is in the intake.
     */
    public boolean intakeSensedBall() {
        return intakeProximity.getState();
    }

    public boolean isGateOpen() {
        if (Robot.isRobotA)
            return DoubleSolenoid.Value.kForward == gateA.get();
        return gateB.get() != IS_GATE_REVERSED;
    }

    public void openGate(boolean open) {
        if (Robot.isRobotA) {
            if (open != IS_GATE_REVERSED)
                gateA.set(DoubleSolenoid.Value.kForward);
            else
                gateA.set(DoubleSolenoid.Value.kReverse);
        } else {
            gateB.set(open != IS_GATE_REVERSED);
        }
    }

    /**
     * Alternative way to open the gate, if booleans are too confusing to work with.
     * OPEN is in the state where the stopper is open, and balls can feed freely.
     * CLOSE is the state where the stopper is closed, and balls can't feed freely.
     * TOGGLE state is ignored.
     *
     * @param state state of the stopper, OPEN / CLOSE
     */
    public void setGate(State state) {
        switch (state) {
            case OPEN:
                openGate(true);
                break;
            case CLOSE:
                openGate(false);
                break;
            case TOGGLE:
                openGate(!isGateOpen());
                break;
        }
    }

    @Deprecated
    public void setPower(double speed) {
        setConveyorPower(speed);
        setFunnelPower(speed);
    }

}
