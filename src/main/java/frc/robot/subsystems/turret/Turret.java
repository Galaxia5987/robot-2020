package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.Utils;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.techfire225.webapp.FireLog;

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
    private double targetAngle;
    private boolean isGoingClockwise = true;
    private double turretBacklash; //The angle in degrees which the turret is off from the motor (clockwise). this angle changes based on the turning direction.

    /**
     * configures the encoder and PID constants.
     */
    public Turret() {
        motor.configFactoryDefault();

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, TALON_TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 2, TALON_TIMEOUT);
        resetEncoder();

        motor.setInverted(IS_MOTOR_INVERTED);
        motor.setSensorPhase(IS_ENCODER_INVERTED);
        motor.config_kP(POSITION_PID_SLOT, KP, TALON_TIMEOUT);
        motor.config_kI(POSITION_PID_SLOT, KI, TALON_TIMEOUT);
        motor.config_kD(POSITION_PID_SLOT, KD, TALON_TIMEOUT);
        motor.config_kF(POSITION_PID_SLOT, KF, TALON_TIMEOUT);

        motor.configAllowableClosedloopError(POSITION_PID_SLOT, unitModel.toTicks(ALLOWABLE_ERROR));

        motor.config_kP(MOTION_MAGIC_PID_SLOT, MOTION_MAGIC_KP, TALON_TIMEOUT);
        motor.config_kI(MOTION_MAGIC_PID_SLOT, MOTION_MAGIC_KI, TALON_TIMEOUT);
        motor.config_kD(MOTION_MAGIC_PID_SLOT, MOTION_MAGIC_KD, TALON_TIMEOUT);
        motor.config_kF(MOTION_MAGIC_PID_SLOT, MOTION_MAGIC_KF, TALON_TIMEOUT);

        motor.configMotionAcceleration(unitModel.toTicks100ms(MOTION_MAGIC_ACCELERATION));
        motor.configMotionCruiseVelocity(unitModel.toTicks100ms(MOTION_MAGIC_CRUISE_VELOCITY));

        motor.configPeakCurrentLimit(PEAK_CURRENT);
        motor.configContinuousCurrentLimit(MAX_CURRENT);
        motor.configPeakCurrentDuration(PEAK_DURATION);
        motor.enableCurrentLimit(true);

        // Configure soft limits for the subsystem.
        motor.configReverseSoftLimitEnable(ENABLE_SOFT_LIMITS, TALON_TIMEOUT);
        motor.configForwardSoftLimitEnable(ENABLE_SOFT_LIMITS, TALON_TIMEOUT);
        motor.configSoftLimitDisableNeutralOnLOS(DISABLE_SOFT_LIMITS_ON_DISCONNECT, TALON_TIMEOUT);
        motor.configReverseSoftLimitThreshold(unitModel.toTicks(ALLOWED_ANGLES.getMinimumDouble()));
        motor.configForwardSoftLimitThreshold(unitModel.toTicks(ALLOWED_ANGLES.getMaximumDouble()));

        motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12.0);

        new WebConstantPIDTalon("turret", KP, KI, KD, KF, motor);
    }

    /**
     * Corrects subsystem's backlash.
     */
    public void correctBacklash() {
        double currentVelocity = unitModel.toVelocity(motor.getSelectedSensorVelocity());
        if (Math.abs(currentVelocity) <= VELOCITY_MINIMUM)
            return;
        boolean changedDirection = !(isGoingClockwise == currentVelocity > 0); // Checks whether the turret switched directions since the last movement
        if (changedDirection) {
            isGoingClockwise = !isGoingClockwise;
            turretBacklash = isGoingClockwise ? BACKLASH_ANGLE / 2. : -BACKLASH_ANGLE / 2.;
        }
    }

    /**
     * get the current angle from the controller
     *
     * @return the angle of the turret
     */
    public double getAngle() {
        return unitModel.toUnits(motor.getSelectedSensorPosition()) - turretBacklash; //subtract backlash to get turret angle, add to get motor angle.
    }

    /**
     * set the position of the turret to the setpoint angle.
     *
     * @param angle setpoint angle.
     */
    public void setAngle(double angle) {
        targetAngle = normalizeSetpoint(getNearestTurretPosition(angle));
        //Use motion magic if target angle is big enough, else use tracking PID.
        if (Math.abs(targetAngle - getAngle()) < CONTROL_MODE_THRESHOLD) {
            setPidSlot(POSITION_PID_SLOT);
            motor.set(ControlMode.Position, unitModel.toTicks(targetAngle)); // Set the position to the target angle plus the backlash the turret creates.
        } else {
            setPidSlot(MOTION_MAGIC_PID_SLOT);
            motor.set(ControlMode.MotionMagic, unitModel.toTicks(targetAngle));
        }

    }

    public void setPidSlot(int slot) {
        motor.selectProfileSlot(slot, 0);
    }

    public double getNearestTurretPosition(double targetAngle) {
        if(targetAngle < (ALLOWED_ANGLES.getMinimumDouble() - 5)) targetAngle = ALLOWED_ANGLES.getMaximumDouble();
        else if(targetAngle > (ALLOWED_ANGLES.getMaximumDouble() + 5)) targetAngle = ALLOWED_ANGLES.getMinimumDouble();
        return targetAngle;
    }

    /**
     * set the position to the current position to stop the turret at the target position.
     */
    public void stop() {
        motor.set(ControlMode.MotionMagic, getAngle());
    }

    /**
     * set the power the turret will turn.
     *
     * @param speed the speed the turret will turn.
     */
    public void setPower(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public boolean isTurretReady() {
        return Math.abs(getAngle() - targetAngle) <= ANGLE_THRESHOLD && !inDeadZone();
    }

    /**
     * runs periodically, updates the constants and resets encoder position if the hall effect is closed
     */
    @Override
    public void periodic() {
        if(BACKLASH_ANGLE != 0)
            correctBacklash();
        double currentAngle = getAngle();
        SmartDashboard.putNumber("turretSetpoint", targetAngle);
        SmartDashboard.putNumber("turretCurrent", currentAngle);
        SmartDashboard.putNumber("turretOutput", motor.getMotorOutputVoltage());
        CustomDashboard.setTurretReady(isTurretReady());
        CustomDashboard.setTurretAngle(currentAngle);

        FireLog.log("turretSetpoint", targetAngle);
        FireLog.log("turretCurrent", currentAngle);
    }

    /**
     * @return whether the current angle is within the turrets limits.
     */
    public boolean inCorrectRange() {
        return ALLOWED_ANGLES.containsDouble(getAngle());
    }

    /**
     * Because of the climbing rods' height there are dead zones where the turret cannot see a target
     * or cannot shoot because the rods are blocking it's path.
     *
     * @return whether the turret is in its dead zone in which it cannot shoot
     */
    public boolean inDeadZone() {
        return DEAD_ZONE_ANGLES.containsDouble(getAngle());
    }

    /**
     * Resets the turret position based on the absolute encoder of the turret.
     * <p>
     * IMPORTANT: since the turrets absolute encoder only reads 360 degrees,
     * we assume the reading to be from -180 to 180. If the turret is reset
     * when it is more than half a rotation from the starting angle, the turret will
     * DESTROY ITSELF... be warned! do not use this midgame!
     */
    public void resetEncoder() {
        double currentPosition = Math.IEEEremainder(
                        Math.floorMod(motor.getSelectedSensorPosition(1), TICKS_PER_ROTATION) -
                                ((ZERO_POSITION + unitModel.toTicks((180 + UNREACHABLE_ANGLE) % 360)) % TICKS_PER_ROTATION),
                TICKS_PER_ROTATION
                ) + unitModel.toTicks(180 + UNREACHABLE_ANGLE) % TICKS_PER_ROTATION;
        motor.setSelectedSensorPosition((int) currentPosition);
    }

    /**
     * Return a normalized constrained in a certain range.
     *
     * @param setpoint the setpoint.
     * @return the normalized setpoint.
     */
    public double normalizeSetpoint(double setpoint) {
        return MathUtil.clamp(setpoint, ALLOWED_ANGLES.getMinimumDouble(), ALLOWED_ANGLES.getMaximumDouble());
    }

}
