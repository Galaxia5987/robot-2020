/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.FalconConfiguration;
import frc.robot.utilities.Utils;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.ghrobotics.lib.debug.FalconDashboard;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.ROBOT_WIDTH;
import static frc.robot.Ports.Drivetrain.*;
import static frc.robot.RobotContainer.navx;

public class Drivetrain extends SubsystemBase {
    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private double[] pidSet = {VELOCITY_PID_SET[0], VELOCITY_PID_SET[1], VELOCITY_PID_SET[2], VELOCITY_PID_SET[3]};
    private UnitModel lowGearUnitModel = new UnitModel(LOW_TICKS_PER_METER);
    private UnitModel highGearUnitModel = new UnitModel(HIGH_TICKS_PER_METER);
    public UnitModel unitModel = lowGearUnitModel;

    /**
     * The gear shifter will be programmed according to the following terms
     * High gear - low torque High speed
     * Low gear - high torque Low speed
     */
    private DoubleSolenoid gearShifterA = null;
    private Solenoid gearShifterB = null;
    private Timer shiftCooldown = new Timer();
    private Timer localizationTimer = new Timer();
    private boolean isShifting = false;


    public Drivetrain() {
        FalconConfiguration motorConfigurations = new FalconConfiguration();

        new WebConstantPIDTalon("drivetrainLeft", pidSet[0], pidSet[1], pidSet[2], pidSet[3], leftMaster);
        new WebConstantPIDTalon("drivetrainRight", pidSet[0], pidSet[1], pidSet[2], pidSet[3], rightMaster);
        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setInverted(RIGHT_MASTER_INVERTED);
        rightSlave.setInverted(RIGHT_SLAVE_INVERTED);
        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);
        motorConfigurations.setNeutralMode(NeutralMode.Brake);
        motorConfigurations.setEnableVoltageCompensation(true);
        motorConfigurations.configureVoltageCompensationSaturation(12);
        motorConfigurations.setPidSet(pidSet[0], pidSet[1], pidSet[2], pidSet[3]);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setSupplyCurrentLimit(40);
        Utils.configAllFalcons(motorConfigurations, rightMaster, rightSlave, leftMaster, leftSlave);
        if(Robot.hasShifter) {
            if (Robot.isRobotA)
                gearShifterA = new DoubleSolenoid(1, SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
            else
                gearShifterB = new Solenoid(1, SHIFTER_PORT);
        }
    }

    public void shiftGear(shiftModes mode) {
        switch (mode) {
            case TOGGLE:
                if (canShiftHigh())
                    shiftHigh();
                else if (canShiftLow())
                    shiftLow();
                break;
            case LOW:
                if (canShiftLow())
                    shiftLow();
                break;
            case HIGH:
                if (canShiftHigh())
                    shiftHigh();
                break;
            default:
                return;
        }
    }

    /**
     * Start the cooldown of the shifter so it won't shift too open
     */
    public void startCooldown() {
        if (getCooldown() == 0.05) {
            shiftCooldown.start();
            isShifting = true;
        }
    }

    /**
     * Resets the cooldown of the shifter and stops it
     */
    public void resetCooldown() {
        shiftCooldown.stop();
        shiftCooldown.reset();
        isShifting = false;
    }

    /**
     * Returns the the time since the last shift
     *
     * @return
     */
    public double getCooldown() {
        return shiftCooldown.get();
    }


    private void shiftHigh() {
        startCooldown();
        if (Robot.isRobotA)
            gearShifterA.set(DoubleSolenoid.Value.kForward);
        else
            gearShifterB.set(true);
    }

    private void shiftLow() {
        startCooldown();
        if (Robot.isRobotA)
            gearShifterA.set(DoubleSolenoid.Value.kReverse);
        else
            gearShifterB.set(false);
    }

    /**
     * Checks if the drivetrain is  able to switch to highgear
     *
     * @return
     */
    private boolean canShiftHigh() {
        return shiftCooldown.get() > SHIFTER_COOLDOWN
                && !isShifting
                && (double) navx.getWorldLinearAccelX() * GRAVITY_ACCELERATION > HIGH_ACCELERATION_THRESHOLD
                && !isShiftedHigh()
                && Math.abs(getLeftVelocity() - getRightVelocity()) < TURNING_TOLERANCE
                && (getLeftVelocity() + getRightVelocity()) / 2 > HIGH_GEAR_MIN_VELOCITY;
    }

    /**
     * Checks if the drivetrain is  able to switch to lowgear
     *
     * @return
     */
    private boolean canShiftLow() {
        return shiftCooldown.get() > SHIFTER_COOLDOWN
                && !isShifting
                && (double) navx.getWorldLinearAccelX() < LOW_ACCELERATION_THRESHOLD
                && !isShiftedLow()
                && Math.abs(getLeftVelocity() - getRightVelocity()) / 2 < TURNING_TOLERANCE
                && leftMaster.getMotorOutputPercent() + rightMaster.getMotorOutputPercent() > LOW_GEAR_MIN_OUTPUT;
    }

    /**
     * @return the velocity of the right motor
     */
    public double getRightVelocity() {
        if (isShiftedLow())
            return lowGearUnitModel.toVelocity(rightMaster.getSelectedSensorVelocity());
        else
            return highGearUnitModel.toUnits(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the left motor
     */
    public double getLeftVelocity() {
        if (isShiftedLow())
            return lowGearUnitModel.toVelocity(leftMaster.getSelectedSensorVelocity());
        else
            return highGearUnitModel.toUnits(leftMaster.getSelectedSensorVelocity());
    }

    public double getLeftPosition(){
        return unitModel.toUnits(leftMaster.getSelectedSensorPosition());
    }

    public double getRightPosition(){
        return unitModel.toUnits(leftMaster.getSelectedSensorPosition());
    }

    /**
     * Indicates whether the shifter is on a high gear
     * @return
     */
    public boolean isShiftedHigh() {
        if (Robot.isRobotA && gearShifterA != null)
            return gearShifterA.get() == DoubleSolenoid.Value.kForward;
        else if (gearShifterB != null)
            return gearShifterB.get();
        else
            return false;
    }

    /**
     * Indicates whether the shifter is on a low gear
     * @return
     */
    public boolean isShiftedLow() {
        return !isShiftedHigh();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360) * (GYRO_INVERTED ? -1 : 1);
    }

    public void setVelocityAndFeedForward(double leftVelocity, double rightVelocity, double leftFF, double rightFF) {
        UnitModel unitModel = isShiftedLow() ? lowGearUnitModel : highGearUnitModel;
        leftMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(leftVelocity), DemandType.ArbitraryFeedForward, leftFF);
        rightMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(rightVelocity), DemandType.ArbitraryFeedForward, rightFF);
    }

    public void setPower(double leftPower, double rightPower){
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        UnitModel unitModel = isShiftedLow() ? lowGearUnitModel : highGearUnitModel;
        unitModel = lowGearUnitModel;

        if (getCooldown() > SHIFTER_COOLDOWN)
            resetCooldown();


    }

    public enum shiftModes {
        TOGGLE,
        HIGH,
        LOW
    }
}
