/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climb extends SubsystemBase {

    private TalonSRX leftClimbMaster = new TalonSRX(Ports.climb.leftClimbMaster);
    private TalonSRX rightClimbMaster = new TalonSRX(Ports.climb.rightClimbMaster);
    private DoubleSolenoid stopper = new DoubleSolenoid(Ports.climb.moduleNumber, Ports.climb.solenoidForward, Ports.climb.solenoidReverse);
    private UnitModel climbUnitModel = new UnitModel(Constants.Climb.TICKS_PER_METER);

    /**
     * Creates a new climb Subsystem.
     */
    public Climb() {
        leftClimbMaster.setInverted(Constants.Climb.LEFT_IS_REVERSED);
        rightClimbMaster.setInverted(Constants.Climb.RIGHT_IS_REVERSED);

        leftClimbMaster.configMotionCruiseVelocity(Constants.Climb.MOTION_MAGIC_VELOCITY);
        rightClimbMaster.configMotionCruiseVelocity(Constants.Climb.MOTION_MAGIC_VELOCITY);

        leftClimbMaster.configMotionAcceleration(Constants.Climb.MOTION_MAGIC_ACCELERATION);
        rightClimbMaster.configMotionAcceleration(Constants.Climb.MOTION_MAGIC_ACCELERATION);

        leftClimbMaster.config_kP(0, Constants.Climb.CLIMB_PIDF[0]);
        rightClimbMaster.config_kP(0, Constants.Climb.CLIMB_PIDF[0]);
        leftClimbMaster.config_kI(0, Constants.Climb.CLIMB_PIDF[1]);
        rightClimbMaster.config_kI(0, Constants.Climb.CLIMB_PIDF[1]);
        leftClimbMaster.config_kD(0, Constants.Climb.CLIMB_PIDF[2]);
        rightClimbMaster.config_kD(0, Constants.Climb.CLIMB_PIDF[2]);
        leftClimbMaster.config_kF(0, Constants.Climb.CLIMB_PIDF[3]);
        rightClimbMaster.config_kF(0, Constants.Climb.CLIMB_PIDF[3]);

        leftClimbMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightClimbMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftClimbMaster.setSensorPhase(Constants.Climb.LEFT_SENSOR_PHASE);
        rightClimbMaster.setSensorPhase(Constants.Climb.RIGHT_SENSOR_PHASE);

        leftClimbMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        rightClimbMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    /**
     * This method release the mechanical stopper from the climb subsystem.
     * This would allow her to rise
     */
    public void releaseStopper() {
        if (isEngaged()) {
            stopper.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * This method engage the mechanical stopper to lock the climb subsystem.
     */
    public void engageStopper() {
        if (!isEngaged())
            stopper.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * This method return if the mechanical stopper is engaged
     *
     * @return the state of the mechanical stopper
     */
    public boolean isEngaged() {
        return stopper.get() == DoubleSolenoid.Value.kForward;
    }


    /**
     * This method move the left side of the climb to certain height
     *
     * @param height the target height of the motor in meters
     */
    public void setLeftHeight(double height) {
        leftClimbMaster.set(ControlMode.MotionMagic, climbUnitModel.toTicks(normalizeSetPoint(height)), DemandType.ArbitraryFeedForward, Constants.Climb.CLIMB_PIDF[3]);
    }

    /**
     * This method move the right side of the climb to certain height
     *
     * @param height the target height of the motor in meters
     */
    public void setRightHeight(double height) {
        rightClimbMaster.set(ControlMode.MotionMagic, climbUnitModel.toTicks(normalizeSetPoint(height)), DemandType.ArbitraryFeedForward, Constants.Climb.CLIMB_PIDF[3]);
    }

    /**
     * @return the current height of the left side of the climb in meters.
     */
    public double getLeftHeight() {
        return climbUnitModel.toUnits(leftClimbMaster.getSelectedSensorPosition());
    }

    /**
     * @return the current height of the right side of the climb in meters.
     */
    public double getRightHeight() {
        return climbUnitModel.toUnits(rightClimbMaster.getSelectedSensorPosition());
    }


    /**
     * @return whether the limit switch on the left side is closed.
     */
    public boolean isLeftOnLimit() {
        return leftClimbMaster.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * @return whether the limit switch on the right side is closed.
     */
    public boolean isRightOnLimit() {
        return rightClimbMaster.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Reset the encoder position to the height of the subsystem.
     */
    public void leftReset() {
        leftClimbMaster.setSelectedSensorPosition(climbUnitModel.toTicks(Constants.Climb.CLIMB_HEIGHT));
    }

    /**
     * Reset the encoder position to the height of the subsystem.
     */
    public void rightReset() {
        rightClimbMaster.setSelectedSensorPosition(climbUnitModel.toTicks(Constants.Climb.CLIMB_HEIGHT));
    }

    /**
     * This method would normalize the setPoint in range.
     * @param setPoint the setPoint.
     * @return the normalized setPoint.
     */
    private double normalizeSetPoint(double setPoint) {
        if (setPoint > Constants.Climb.CLIMB_HEIGHT) {
            return Constants.Climb.CLIMB_HEIGHT;
        } else if (setPoint < 0) {
            return 0;
        }
        return setPoint;
    }

    @Override
    public void periodic() {

        //Reset if the limit switch is pressed.
        if (isLeftOnLimit()) {
            leftReset();
        }else if (isRightOnLimit()) {
            rightReset();
        }

        //Engage the stopper to prevent the subsystem from exceeding the limits.
        if (isRightOnLimit() || isLeftOnLimit()) {
            engageStopper();
        }

        //Limit the motors to a certain range.
        if (getLeftHeight() <=0 ){
            setLeftHeight(0);
        }else if (getLeftHeight() >= Constants.Climb.CLIMB_HEIGHT){
            setLeftHeight(Constants.Climb.CLIMB_HEIGHT);
        }

        if (getRightHeight() <=0 ){
            setRightHeight(0);
        }else if (getRightHeight() >= Constants.Climb.CLIMB_HEIGHT){
            setRightHeight(Constants.Climb.CLIMB_HEIGHT);
        }
    }
}
