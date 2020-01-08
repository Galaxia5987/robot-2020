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
            adjustEncoderPosition();
        }
    }


    /**
     * get the current angle from the controller
     *
     * @return the angle of the turret
     */
    public double getAngle() {
        return convertTicksToDegrees(master.getSelectedSensorPosition());
    }

    /**
     * change the angle to the desired angle,
     * the value can be between 0 to 360 degrees.
     *
     * @param targetAngle the desired angle.
     * @return return the target angle in ticks.
     */
    private double setTargetAngle(double targetAngle) {
        targetAngle = (targetAngle + 720) % 360; //To insure that the targetAngle is between 0-360, we add 720 to prevent negative modulo operations.
        targetAngle = constrain(MINIMUM_ANGLE, targetAngle, MAXIMUM_ANGLE);
        return convertDegreesToTicks(targetAngle);
    }


    /**
     * apply power to the controller to move the turret.
     *
     * @param angle the desired angle
     */
    public void moveTurret(double angle) {
        master.set(ControlMode.MotionMagic, setTargetAngle(angle));
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
    public void adjustEncoderPosition() {
        master.setSelectedSensorPosition(convertDegreesToTicks(HALL_EFFECT_POSITION), 0, TALON_TIMEOUT);
    }

    /**
     * @param minimum the minimum angle the turret can turn
     * @param angle the target angle
     * @param maximum the maximum angle that the turret can turn
     * @return an angle that satisfies the constrain
     */
    private double constrain(double minimum, double angle, double maximum) {
        return Math.min(maximum, Math.max(minimum, angle));
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