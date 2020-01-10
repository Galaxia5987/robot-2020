package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;

import static frc.robot.Constants.Turret.*;
import static frc.robot.Ports.Turret.*;

/**
 * @author Adam & Barel
 * @version 1.0
 * <p>
 * this class includes methods for the Turret.
 * {@using TalonSRX}
 * {@using Relative Encoder}
 * {@using Hall Effect}
 */
public class Turret extends SubsystemBase {
    public TurretAlgorithms algorithms = new TurretAlgorithms();
    private TalonSRX master = new TalonSRX(MASTER);
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("turret");
    private NetworkTableEntry kPentry = table.getEntry("kP");
    private NetworkTableEntry kIentry = table.getEntry("kI");
    private NetworkTableEntry kDentry = table.getEntry("kD");
    private NetworkTableEntry kFentry = table.getEntry("kF");

    /**
     * configures the encoder and PID constants.
     */
    public Turret() {
        master.configFactoryDefault();
        master.setInverted(IS_MASTER_INVERTED);
        master.setSensorPhase(IS_ENCODER_INVERTED);
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        master.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        master.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        master.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        master.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);
        master.configMotionAcceleration(CRUISE_ACCELERATION);
        master.configMotionCruiseVelocity(CRUISE_VELOCITY);
        master.configPeakCurrentLimit(MAX_CURRENT);
        master.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        master.setSelectedSensorPosition((int) HALL_EFFECT_POSITION, 0, TALON_TIMEOUT);
    }

    /**
     * updates the turret PID constants and configures the controller PID
     */
    public void updateConstants() {
        Utils.setValue(kPentry, KP);
        Utils.setValue(kIentry, KP);
        Utils.setValue(kDentry, KP);
        Utils.setValue(kFentry, KP);
        master.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        master.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        master.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        master.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);
    }

    @Override
    public void periodic() {
        updateConstants();
        if(getHallEffect()) {
            resetEncoderPosition();
        }
    }


    /**
     * get the current angle from the controller
     *
     * @return the angle of the turret
     */
    public double getEncoderPosition() {
        return master.getSelectedSensorPosition();
    }

    /**
     * change the angle to the desired angle,
     * the value can be between -360 to 360 degrees.
     *
     * @param targetAngle the desired angle.
     * @return return the target angle in ticks.
     */
    public double setTurretAngle(double targetAngle) {
        return convertDegreesToTicks(algorithms.setTurretAngle(targetAngle, getEncoderPosition(), MINIMUM_POSITION, MAXIMUM_POSITION));
    }

    public double center(double minimum, double maximum) {
        return convertDegreesToTicks(algorithms.center(getEncoderPosition(), minimum, maximum));
    }

    /**
     * apply power to the controller to move the turret.
     *
     * @param angle the desired angle
     */
    public void moveTurret(double angle) {
        master.set(ControlMode.MotionMagic, setTurretAngle(angle));
    }

    /**
     * set the speed of the motor to 0.
     */
    public void stop() {
        master.set(ControlMode.PercentOutput, 0);
    }

    /**
     * @return return if the state of the Hall Effect sensor is Closed.
     */
    public boolean getHallEffect() {
        return master.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * set encoder position to the Hall Effect position.
     */
    public void resetEncoderPosition() {
        master.setSelectedSensorPosition(convertDegreesToTicks(HALL_EFFECT_POSITION), 0, TALON_TIMEOUT);
    }


    /**
     * convert the angle to ticks so the controller will apply the right amount of power on the turret.
     *
     * @param degrees the degrees to convert.
     * @return the degrees converted to ticks.
     */
    private int convertDegreesToTicks(double degrees) {
        return (int) (degrees * TICKS_PER_DEGREE);
    }

    /**
     * convert the ticks from the controller to angle so human can understand.
     *
     * @param ticks the ticks to convert.
     * @return the ticks converted to ticks
     */
    private double convertTicksToDegrees(int ticks) {
        return ticks / TICKS_PER_DEGREE;
    }

    /**
     * resets the encoder position to 0
     */
    public void reset() {
        master.setSelectedSensorPosition(0);
    }
}