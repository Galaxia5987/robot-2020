/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.FalconConfiguration;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Ports.Drivetrain.*;
import static frc.robot.subsystems.Drivetrain.Drivetrain.shiftModes.TOGGLE;

public class Drivetrain extends SubsystemBase {


    private static AHRS navx = new AHRS(SPI.Port.kMXP);
    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private FalconConfiguration configurations = new FalconConfiguration();
    private double[] pidSet = {Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD, Constants.Drivetrain.KF};
    private UnitModel drivetrainModel = new UnitModel(TICKS_PER_METER);
    /**
     * The gear shifter will be programmed according to the following terms
     * High gear - low torque High speed
     * Low gear - high torque Low speed
     */
    private DoubleSolenoid AgearShifter = new DoubleSolenoid(1, SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
    private Solenoid BgearShifter = new Solenoid(1, SHIFTER_PORT);
    private Timer shiftCooldown = new Timer();
    private boolean isShifting = false;


    public Drivetrain() {
        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);
        configurations.setNeutralMode(NeutralMode.Coast);
        configurations.setEnableVoltageCompensation(true);
        configurations.setPidSet(pidSet);
        configurations.setEnableCurrentLimit(true);
        configurations.setEnableCurrentLimit(true);
        configurations.setSupplyCurrentLimit(40);
        UtilityFunctions.configAllFalcons(configurations, rightMaster, rightSlave, leftMaster, leftSlave);

    }

    public void shiftGear(shiftModes mode) {
        if (Robot.isRobotA) {
            switch (mode) {
                case TOGGLE:
                    if (isShiftedHigh() && canShiftHigh())
                        AgearShifter.set(DoubleSolenoid.Value.kForward);

                    else if (canShiftLow())
                        AgearShifter.set(DoubleSolenoid.Value.kReverse);
                case LOW:
                    if (canShiftLow())
                        AgearShifter.set(DoubleSolenoid.Value.kReverse);
                case HIGH:
                    if (isShiftedHigh() && canShiftHigh())
                        AgearShifter.set(DoubleSolenoid.Value.kForward);
                default:
                    throw new IllegalStateException("Unexpected value: " + mode);
            }
        } else {
            switch (mode) {
                case TOGGLE:
                    if (isShiftedHigh() && canShiftHigh())
                        BgearShifter.set(true);
                    else if (canShiftLow())
                        BgearShifter.set(false);
                    break;
                case HIGH:
                    if (isShiftedHigh() && canShiftHigh())
                        BgearShifter.set(true);
                    break;
                case LOW:
                    if (canShiftLow())
                        BgearShifter.set(false);
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + mode);

            }
        }
    }

    /**
     * Starts the cooldown of the shifter so it won't rapidly shift
     */
    public void startCooldown(){
        if (getCooldown() == 0) {
            shiftCooldown.start();
            isShifting = true;
        }
    }

    /**
     * Resets the cooldown of the shifter and stops it
     */
    public void resetCooldown(){
        shiftCooldown.stop();
        shiftCooldown.reset();
        isShifting = false;
    }

    /**
     * Returns the the time since the last shift
     * @return
     */
    public double getCooldown(){
        return shiftCooldown.get();
    }


    /**
     * Checks if the drivetrain is  able to switch to highgear
     * @return
     */
    private boolean canShiftHigh() {
        return shiftCooldown.get() > SHIFTER_COOLDOWN
                && !isShifting
                && (double) navx.getRawAccelX() > HIGH_ACCELERATION_THRESHOLD
                && !isShiftedHigh()
                && Math.abs(getLeftVelocity() - getRightVelocity()) < TURNING_TOLERANCE
                && (getLeftVelocity() + getRightVelocity()) / 2 > HIGH_GEAR_MIN_VELOCITY;
    }

    /**
     * Checks if the drivetrain is  able to switch to lowgear
     * @return
     */
    private boolean canShiftLow() {
        return shiftCooldown.get() > SHIFTER_COOLDOWN
                && !isShifting
                && (double) navx.getRawAccelX() > LOW_ACCELERATION_THRESHOLD
                && !isShiftedLow()
                && Math.abs(getLeftVelocity() - getRightVelocity()) / 2 < TURNING_TOLERANCE
                && leftMaster.getMotorOutputPercent() + rightMaster.getMotorOutputPercent() > LOW_GEAR_MIN_OUTPUT;
    }

    /**
     * @return the velocity of the right motor
     */
    private double getRightVelocity() {
        return drivetrainModel.toUnits(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the left motor
     */
    private double getLeftVelocity() {
        return drivetrainModel.toUnits(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * Indicates whether the shifter is on a high gear
     * @return
     */
    public boolean isShiftedHigh() {
        if (Robot.isRobotA)
            return AgearShifter.get() == DoubleSolenoid.Value.kForward;
        else
            return BgearShifter.get();
    }

    /**
     * Indicates whether the shifter is on a low gear
     * @return
     */
    public boolean isShiftedLow() {
        if (Robot.isRobotA)
            return AgearShifter.get() == DoubleSolenoid.Value.kReverse;
        else
            return !BgearShifter.get();
    }


    @Override
    public void periodic() {
        if (getCooldown() > SHIFTER_COOLDOWN)
            resetCooldown();
        // This method will be called once per scheduler run
    }

    public enum shiftModes{
        TOGGLE,
        HIGH,
        LOW
    }
}
