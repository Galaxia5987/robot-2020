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
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.TALON_TIMEOUT;
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
    private final NetworkTableEntry kPentry = table.getEntry("kP");
    private final NetworkTableEntry kIentry = table.getEntry("kI");
    private final NetworkTableEntry kDentry = table.getEntry("kD");
    private final NetworkTableEntry kFentry = table.getEntry("kF");
    private TalonSRX motor = new TalonSRX(MOTOR);
    private UnitModel unitModel = new UnitModel(TICKS_PER_DEGREE);

    /**
     * configures the encoder and PID constants.
     */
    public Turret() {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        motor.setInverted(IS_MOTOR_INVERTED);
        motor.setSensorPhase(IS_ENCODER_INVERTED);
        motor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        motor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        motor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        motor.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);
        motor.configMotionAcceleration(MOTION_MAGIC_ACCELERATION);
        motor.configMotionCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY);
        motor.configPeakCurrentLimit(MAX_CURRENT);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.setSelectedSensorPosition((int) HALL_EFFECT_POSITION_1, 0, TALON_TIMEOUT);
    }


    /**
     * runs periodically, updates the constants and resets encoder position if the hall effect is closed
     */
    @Override
    public void periodic() {
        if (isLimitSwitchClosed()) {
            resetEncoderPosition();
        }
    }


    /**
     * get the current angle from the controller
     *
     * @return the angle of the turret
     */
    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    /**
     * change the angle to the desired angle,
     * the value can be between -360 to 360 degrees.
     *
     * @param targetAngle the desired angle.
     * @return return the target angle in ticks.
     */
    public double getNearestTurretPosition(double targetAngle, double currentPosition, double MINIMUM_POSITION, double MAXIMUM_POSITION) {
        targetAngle = mathFloor(targetAngle);
        double[] positions = {targetAngle - 360, targetAngle, targetAngle + 360}; // An array of all possible target positions
        double targetPosition = currentPosition;
        double shortestDistance = Double.MAX_VALUE;
        for (double targetPos : positions) { // for each possible position
            if (targetPos < MINIMUM_POSITION || targetPos > MAXIMUM_POSITION) // if the position is out of boundaries
                continue;
            if (Math.abs(targetPos - currentPosition) < shortestDistance) // if the calculated distance is less than the current shortest distance
            {
                shortestDistance = Math.abs(targetPos - currentPosition);
                targetPosition = targetPos;
            }
        }
        return unitModel.toTicks(targetPosition);
    }

    /**
     * sets the target angle to the corresponding angle between 0 and 360.
     *
     * @param targetAngle the setpoint angle.
     * @return the angle between 0 and 360.
     */
    private double mathFloor(double targetAngle) {
        targetAngle %= 360;
        targetAngle += 360;
        targetAngle %= 360;
        return targetAngle;
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
        return unitModel.toTicks(currentPosition);
    }

    /**
     * set the position of the turret to the setpoint angle.
     *
     * @param angle setpoint angle.
     */
    public void setAngle(double angle) {
        double targetAngle = getNearestTurretPosition(angle, getEncoderPosition(), MINIMUM_POSITION, MAXIMUM_POSITION);
        motor.set(ControlMode.MotionMagic, unitModel.toTicks(targetAngle));
    }

    /**
     * sets the position of the turret to the joystick position.
     *
     * @param position the setpoint position indicated by the joystick.
     */
    public void setJoystickPosition(double position) {
        motor.set(ControlMode.Position, unitModel.toTicks(position));
    }

    /**
     * set the position to the current position to stop the turret at the target position.
     */
    public void stop() {
        motor.set(ControlMode.MotionMagic, getEncoderPosition());
    }

    /**
     * @return return if the state of the Hall Effect sensor is Closed.
     */
    public boolean isLimitSwitchClosed() {
        return motor.getSensorCollection().isRevLimitSwitchClosed();
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
        motor.setSelectedSensorPosition(unitModel.toTicks(resetAngle), 0, TALON_TIMEOUT);
    }

}