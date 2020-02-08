package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
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
    private NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("turret");
    private NetworkTableEntry visionAngle = visionTable.getEntry("targetYaw");
    private NetworkTableEntry visionValid = visionTable.getEntry("isValid");
    private double targetAngle;
    private boolean isGoingClockwise = true;
    private double turretBacklash; //The angle in degrees which the turret is off from the motor (clockwise). this angle changes based on the turning direction.
    /**
     * configures the encoder and PID constants.
     */
    public Turret() {
        motor.configFactoryDefault();

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, TALON_TIMEOUT); // Todo: check if this experimental idea works.
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        resetEncoder();

        motor.setInverted(IS_MOTOR_INVERTED);
        motor.setSensorPhase(IS_ENCODER_INVERTED);
        motor.config_kP(0, KP, TALON_TIMEOUT);
        motor.config_kI(0, KI, TALON_TIMEOUT);
        motor.config_kD(0, KD, TALON_TIMEOUT);
        motor.config_kF(0, KF, TALON_TIMEOUT);
        motor.configMotionAcceleration(unitModel.toTicks100ms(MOTION_MAGIC_ACCELERATION));
        motor.configMotionCruiseVelocity(unitModel.toTicks100ms(MOTION_MAGIC_CRUISE_VELOCITY));
        motor.configPeakCurrentLimit(0);
        motor.configContinuousCurrentLimit(MAX_CURRENT);
        motor.configPeakCurrentDuration(0);
        motor.enableCurrentLimit(true);

        motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        new WebConstantPIDTalon("turret", KP, KI, KD, KF, motor);
    }

    /**
     * Corrects subsystem's backlash.
     */
    public void correctBacklash(){
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
        targetAngle = getNearestTurretPosition(angle, getAngle(), MINIMUM_POSITION, MAXIMUM_POSITION);
        motor.set(ControlMode.MotionMagic, unitModel.toTicks(targetAngle + turretBacklash)); //Set the position to the target angle plus the backlash the turret creates.
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
     * set the position to the current position to stop the turret at the target position.
     */
    public void stop() {
        motor.set(ControlMode.MotionMagic, getAngle());
    }

    /**
     * @return the angle to the target from the vision network table.
     */
    public double getVisionAngle(){
        return visionAngle.getDouble(0);
    }

    /**
     * @return whether the vision has calculated the angle to the target.
     */
    public boolean hasVisionAngle() {
        return visionValid.getBoolean(false);
    }

    /**
     * set the power the turret will turn.
     * @param speed the speed the turret will turn.
     */
    public void setPower(double speed){
        motor.set(ControlMode.PercentOutput, speed);   
    }
    
    public boolean isTurretReady(){
        return Math.abs(getAngle() - targetAngle) <= ANGLE_THRESHOLD;
    }
    /**
     * runs periodically, updates the constants and resets encoder position if the hall effect is closed
     */
    @Override
    public void periodic() {
        correctBacklash();
        SmartDashboard.putNumber("turretSetpoint", targetAngle);
        SmartDashboard.putNumber("turretCurrent", getAngle());
        SmartDashboard.putNumber("turretOutput", motor.getMotorOutputVoltage());
        FireLog.log("turretSetpoint", targetAngle);
        FireLog.log("turretCurrent", getAngle());
    }

    /**
     * @return whether the current angle is within the turrets limits.
     */
    public boolean inCorrectRange() {
        return getAngle() > MINIMUM_POSITION && getAngle() < MAXIMUM_POSITION;
    }

    /**
     * Resets the turret position based on the absolute encoder of the turret.
     *
     * IMPORTANT: since the turrets absolute encoder only reads 360 degrees,
     * we assume the reading to be from -180 to 180. If the turret is reset
     * when it is more than half a rotation from the starting angle, the turret will
     * DESTROY ITSELF... be warned! do not use this midgame!
     */
    public void resetEncoder(){
        double currentPosition = unitModel.toTicks(STARTING_ANGLE) + Math.IEEEremainder(motor.getSelectedSensorPosition(1) - Constants.Turret.STARTING_POSITION, unitModel.toTicks(360));
        motor.setSelectedSensorPosition((int)currentPosition);
    }
}
