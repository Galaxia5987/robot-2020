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

public class Climber extends SubsystemBase {

    private TalonSRX leftMotor = new TalonSRX(Ports.climber.leftMotor);
    private TalonSRX rightMotor = new TalonSRX(Ports.climber.rightMotor);
    private DoubleSolenoid stopper = new DoubleSolenoid(Ports.moduleNumber, Ports.climber.stopperForward, Ports.climber.stopperReverse);
    private UnitModel climbUnitModel = new UnitModel(Constants.Climb.TICKS_PER_METER);

    /**
     * Creates a new climb Subsystem.
     */
    public Climber() {
        leftMotor.setInverted(Constants.Climb.IS_LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(Constants.Climb.IS_RIGHT_MOTOR_INVERTED);

        leftMotor.configMotionCruiseVelocity(Constants.Climb.MOTION_MAGIC_VELOCITY);
        rightMotor.configMotionCruiseVelocity(Constants.Climb.MOTION_MAGIC_VELOCITY);

        leftMotor.configMotionAcceleration(Constants.Climb.MOTION_MAGIC_ACCELERATION);
        rightMotor.configMotionAcceleration(Constants.Climb.MOTION_MAGIC_ACCELERATION);

        leftMotor.config_kP(0, Constants.Climb.CLIMB_PIDFA[0]);
        rightMotor.config_kP(0, Constants.Climb.CLIMB_PIDFA[0]);
        leftMotor.config_kI(0, Constants.Climb.CLIMB_PIDFA[1]);
        rightMotor.config_kI(0, Constants.Climb.CLIMB_PIDFA[1]);
        leftMotor.config_kD(0, Constants.Climb.CLIMB_PIDFA[2]);
        rightMotor.config_kD(0, Constants.Climb.CLIMB_PIDFA[2]);
        leftMotor.config_kF(0, Constants.Climb.CLIMB_PIDFA[3]);
        rightMotor.config_kF(0, Constants.Climb.CLIMB_PIDFA[3]);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftMotor.setSensorPhase(Constants.Climb.IS_LEFT_SENSOR_INVERTED);
        rightMotor.setSensorPhase(Constants.Climb.IS_RIGHT_SENSOR_INVERTED);

        leftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        rightMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    /**
     * Release the mechanical stopper of the climber.
     * This would allow it to extend.
     */
    public void releaseStopper() {
        if (isEngaged()) {
            stopper.set(DoubleSolenoid.Value.kReverse);
        }
    }

    /**
     * Engage the mechanical stopper to lock the climber.
     */
    public void engageStopper() {
        if (!isEngaged())
            stopper.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Return whether the mechanical stopper is engaged.
     *
     * @return the state of the mechanical stopper
     */
    public boolean isEngaged() {
        return stopper.get() == DoubleSolenoid.Value.kForward;
    }


    /**
     * Move the left side of the climber to a given height.
     *
     * @param height the setpoint height of the left elevator in meters
     */
    public void setLeftHeight(double height) {
        leftMotor.set(ControlMode.MotionMagic, climbUnitModel.toTicks(normalizeSetPoint(height)), DemandType.ArbitraryFeedForward, Constants.Climb.CLIMB_PIDFA[4]);
    }

    /**
     * Move the right side of the climber to a given height.
     *
     * @param height the setpoint height of the right elevator in meters
     */
    public void setRightHeight(double height) {
        rightMotor.set(ControlMode.MotionMagic, climbUnitModel.toTicks(normalizeSetPoint(height)), DemandType.ArbitraryFeedForward, Constants.Climb.CLIMB_PIDFA[4]);
    }

    /**
     * @return the current height of the left side of the climber in meters.
     */
    public double getLeftHeight() {
        return climbUnitModel.toUnits(leftMotor.getSelectedSensorPosition());
    }

    /**
     * @return the current height of the right side of the climber in meters.
     */
    public double getRightHeight() {
        return climbUnitModel.toUnits(rightMotor.getSelectedSensorPosition());
    }


    /**
     * @return whether the left elevator reached its limit.
     */
    public boolean isLeftOnLimit() {
        return leftMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * @return whether the right elevator reached its limit.
     */
    public boolean isRightOnLimit() {
        return rightMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Reset the encoder position to the height of the subsystem.
     */
    public void leftReset() {
        leftMotor.setSelectedSensorPosition(climbUnitModel.toTicks(Constants.Climb.CLIMB_HEIGHT));
    }

    /**
     * Reset the encoder position to the height of the subsystem.
     */
    public void rightReset() {
        rightMotor.setSelectedSensorPosition(climbUnitModel.toTicks(Constants.Climb.CLIMB_HEIGHT));
    }

    /**
     * Return a normalized constrained in a certain range.
     * @param setpoint the setpoint.
     * @return the normalized setpoint.
     */
    private double normalizeSetPoint(double setpoint) {
        if (setpoint > Constants.Climb.CLIMB_HEIGHT) {
            return Constants.Climb.CLIMB_HEIGHT;
        } else if (setpoint < 0) {
            return 0;
        }
        return setpoint;
    }

    @Override
    public void periodic() {

        //Reset if the limit switch is pressed.
        if (isLeftOnLimit()) {
            leftReset();
        }

        if (isRightOnLimit()) {
            rightReset();
        }

        // Engage the stopper to prevent the subsystem from exceeding the limits.
        if (isRightOnLimit() || isLeftOnLimit()) {
            engageStopper();
        }

        // Limit the elevators to a certain range.
        double leftHeight = normalizeSetPoint(getLeftHeight());
        double rightHeight = normalizeSetPoint(getRightHeight());
        setLeftHeight(leftHeight);
        setRightHeight(rightHeight);
        
    }
}
