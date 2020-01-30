package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.Utils;

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
    private TalonSRX motor = new TalonSRX(MOTOR);
    private UnitModel unitModel = new UnitModel(TICKS_PER_DEGREE);
    private NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("turret");
    private NetworkTableEntry visionAngle = visionTable.getEntry("visionAngle");
    private double targetAngle;
    /**
     * configures the encoder and PID constants.
     */
    public Turret() {
        motor.configFactoryDefault();
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        motor.setInverted(IS_MOTOR_INVERTED);
        motor.setSensorPhase(IS_ENCODER_INVERTED);
        motor.config_kP(0, KP, TALON_TIMEOUT);
        motor.config_kI(0, KI, TALON_TIMEOUT);
        motor.config_kD(0, KD, TALON_TIMEOUT);
        motor.config_kF(0, KF, TALON_TIMEOUT);
        motor.configMotionAcceleration(MOTION_MAGIC_ACCELERATION);
        motor.configMotionCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY);
        motor.configPeakCurrentLimit(MAX_CURRENT);
    }

    /**
     * runs periodically, updates the constants and resets encoder position if the hall effect is closed
     */
    @Override
    public void periodic() {
    }

    public void setPower(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * set the position to the current position to stop the turret at the target position.
     */
    public void stop() {
        targetAngle = getAngle();
    }


    /**
     * get the current angle from the controller
     *
     * @return the angle of the turret
     */
    public double getAngle() {
        return unitModel.toUnits(motor.getSelectedSensorPosition());
    }

    /**
     * change the angle to the desired angle,
     * the value can be between -360 to 360 degrees.
     *
     * @param targetAngle the desired angle.
     * @return return the target angle in ticks.
     */
    public double getNearestTurretPosition(double targetAngle, double currentPosition, double MINIMUM_POSITION, double MAXIMUM_POSITION) {
        targetAngle = Utils.floorMod(targetAngle, 360);
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
        return targetPosition;
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
        return currentPosition;
    }

    /**
     * set the position to the current position to stop the turret at the target position.
     */
    public void stop() {
        motor.set(ControlMode.MotionMagic, getAngle());
    }
      * set the position of the turret to the setpoint angle.
     *
     * @param angle setpoint angle.
     */
    public void setAngle(double angle) {
        targetAngle = getNearestTurretPosition(angle, getAngle(), MINIMUM_POSITION, MAXIMUM_POSITION);
        motor.set(ControlMode.MotionMagic, unitModel.toTicks(targetAngle));
    }

    public double getVisionAngle() {
        return visionAngle.getDouble(0);
    }

    public void setPower(double speed) {
        if (speed > 0 && getAngle() >= MAXIMUM_POSITION || speed < 0 && getAngle() <= MINIMUM_POSITION) {
            speed = 0;
        }
        motor.set(ControlMode.PercentOutput, speed);
    }
  public boolean isTurretReady(){
        return Math.abs(getAngle() - targetAngle) <= ANGLE_THRESHOLD;
    }

}