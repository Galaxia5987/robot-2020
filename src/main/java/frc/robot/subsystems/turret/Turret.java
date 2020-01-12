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
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("turret");
    private TalonSRX master = new TalonSRX(MASTER);
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
        master.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        master.setSelectedSensorPosition((int) HALL_EFFECT_POSITION_1, 0, TALON_TIMEOUT);
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
        if (isHallEffectClosed()) {
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
    public double getNearestTurretPosition(double targetAngle, double currentPosition, double MINIMUM_POSITION, double MAXIMUM_POSITION) throws Exception {
        targetAngle %= 360;
        targetAngle += 360;
        targetAngle %= 360; //Ensure that targetAngle is a number between 0-360.
        double[] positions = {targetAngle - 360, targetAngle, targetAngle + 360}; // An array of all possible target positions
        double targetPosition = Double.NaN;
        double shortestDistance = Double.MAX_VALUE;
        for (double _targetPos : positions) { // for each possible position
            if (_targetPos < MINIMUM_POSITION || _targetPos > MAXIMUM_POSITION) // if the position is out of boundaries
                continue;
            if (Math.abs(_targetPos - currentPosition) < shortestDistance) // if the calculated distance is less than the current shortest distance
            {
                shortestDistance = Math.abs(_targetPos - currentPosition);
                targetPosition = _targetPos;
            }
        }
        if(targetPosition == Double.NaN)
            throw new Exception("Can't reach specified angle {}");
        return convertDegreesToTicks(targetPosition);
    }


    /**
     * @return the same position rotated 360 degrees or the current position in ticks
     */
    public double center(double currentPosition, double MINIMUM_POSITION, double MAXIMUM_POSITION) {
        double middle = (MINIMUM_POSITION + MAXIMUM_POSITION) / 2;
        if (currentPosition > (180 + middle)) {
            currentPosition -= 360;
        } else if (currentPosition < (-180 + middle)) {
            currentPosition += 360;
        }
        return convertDegreesToTicks(currentPosition);
    }

    /**
     * apply power to the controller to move the turret.
     *
     * @param angle setpoint angle
     */
    public void setPosition(double angle) {
        double targetTurretPosition = 0;
        try {
            targetTurretPosition = getNearestTurretPosition(angle, getEncoderPosition(), MINIMUM_POSITION, MAXIMUM_POSITION);
        } catch (Exception e) {
            return;
        }
        master.set(ControlMode.MotionMagic, targetTurretPosition);
    }

    /**
     * set the position to the target position to stop the turret at the target position.
     */
    public void stop(double targetPosition) {
        master.set(ControlMode.Position, targetPosition);
    }

    /**
     * @return return if the state of the Hall Effect sensor is Closed.
     */
    public boolean isHallEffectClosed() {
        return master.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * set encoder position to the nearest Hall Effect position.
     */
    public void resetEncoderPosition() {
        double resetAngle;
        if (Math.abs(getEncoderPosition() - HALL_EFFECT_POSITION_1) < Math.abs(getEncoderPosition() - HALL_EFFECT_POSITION_2))
            resetAngle = HALL_EFFECT_POSITION_1;
        else
            resetAngle = HALL_EFFECT_POSITION_2;
        master.setSelectedSensorPosition(convertDegreesToTicks(resetAngle), 0, TALON_TIMEOUT);
    }


    /**
     * convert the angle to ticks so the controller will apply the right amount of power on the turret.
     *
     * @param degrees the degrees to convert.
     * @return the degrees converted to ticks.
     */
    public int convertDegreesToTicks(double degrees) {
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