/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.FalconConfiguration;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Ports.Drivetrain.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.navx;

public class Drivetrain extends SubsystemBase {


    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private FalconConfiguration configurations = new FalconConfiguration();
    private double[] pidSet = {VELOCITY_PID_SET[0], VELOCITY_PID_SET[1], VELOCITY_PID_SET[2], VELOCITY_PID_SET[3]};
    private UnitModel lowGearUnitModel = new UnitModel(LOW_TICKS_PER_METER);
    private UnitModel highGearUnitModel = new UnitModel(HIGH_TICKS_PER_METER);
    /**
     * The gear shifter will be programmed according to the following terms
     * High gear - low torque High speed
     * Low gear - high torque Low speed
     */
    private DoubleSolenoid gearShifterA;
    private Solenoid gearShifterB;
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
        if (Robot.isRobotA)
            gearShifterA = new DoubleSolenoid(1, SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
        else
            gearShifterB = new Solenoid(1, SHIFTER_PORT);

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

    private void shiftHigh(){
        drivetrain.startCooldown();
        if(Robot.isRobotA)
            gearShifterA.set(DoubleSolenoid.Value.kForward);
        else
            gearShifterB.set(true);
    }

    private void shiftLow(){
        drivetrain.startCooldown();
        if(Robot.isRobotA)
            gearShifterA.set(DoubleSolenoid.Value.kReverse);
        else
            gearShifterB.set(false);
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
                && (double) navx.getRawAccelX() < LOW_ACCELERATION_THRESHOLD
                && !isShiftedLow()
                && Math.abs(getLeftVelocity() - getRightVelocity()) / 2 < TURNING_TOLERANCE
                && leftMaster.getMotorOutputPercent() + rightMaster.getMotorOutputPercent() > LOW_GEAR_MIN_OUTPUT;
    }

    /**
     * @return the velocity of the right motor
     */
    private double getRightVelocity() {
        if (isShiftedLow())
            return lowGearUnitModel.toUnits(rightMaster.getSelectedSensorVelocity());
        else
            return highGearUnitModel.toUnits(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the left motor
     */
    private double getLeftVelocity() {
        if (isShiftedLow())
            return lowGearUnitModel.toUnits(leftMaster.getSelectedSensorVelocity());
        else
            return highGearUnitModel.toUnits(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * Indicates whether the shifter is on a high gear
     * @return
     */
    public boolean isShiftedHigh() {
        if (Robot.isRobotA)
            return gearShifterA.get() == DoubleSolenoid.Value.kForward;
        else
            return gearShifterB.get();
    }

    /**
     * Indicates whether the shifter is on a low gear
     * @return
     */
    public boolean isShiftedLow() {
        return !isShiftedHigh();
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
