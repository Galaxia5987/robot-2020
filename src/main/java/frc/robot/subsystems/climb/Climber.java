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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;

public class Climber extends SubsystemBase {

    private final TalonSRX leftMotor = new TalonSRX(Ports.climber.LEFT_MOTOR);
    private final TalonSRX rightMotor = new TalonSRX(Ports.climber.RIGHT_MOTOR);
    private final DoubleSolenoid stopper = new DoubleSolenoid(Ports.climber.STOPPER_FORWARD, Ports.climber.STOPPER_REVERSE);
    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_METER);

    /**
     * Creates a new climb Subsystem.
     */
    public Climber() {
        leftMotor.setInverted(Ports.climber.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(Ports.climber.RIGHT_MOTOR_INVERTED);

        leftMotor.configMotionCruiseVelocity(Constants.Climber.MOTION_MAGIC_VELOCITY);
        rightMotor.configMotionCruiseVelocity(Constants.Climber.MOTION_MAGIC_VELOCITY);

        leftMotor.configMotionAcceleration(Constants.Climber.MOTION_MAGIC_ACCELERATION);
        rightMotor.configMotionAcceleration(Constants.Climber.MOTION_MAGIC_ACCELERATION);

        leftMotor.config_kP(0, Constants.Climber.CLIMB_PIDF[0]);
        rightMotor.config_kP(0, Constants.Climber.CLIMB_PIDF[0]);
        leftMotor.config_kI(0, Constants.Climber.CLIMB_PIDF[1]);
        rightMotor.config_kI(0, Constants.Climber.CLIMB_PIDF[1]);
        leftMotor.config_kD(0, Constants.Climber.CLIMB_PIDF[2]);
        rightMotor.config_kD(0, Constants.Climber.CLIMB_PIDF[2]);
        leftMotor.config_kF(0, Constants.Climber.CLIMB_PIDF[3]);
        rightMotor.config_kF(0, Constants.Climber.CLIMB_PIDF[3]);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftMotor.setSensorPhase(Ports.climber.LEFT_ENCODER_INVERTED);
        rightMotor.setSensorPhase(Ports.climber.RIGHT_ENCODER_INVERTED);

        leftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        rightMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        leftMotor.configClosedloopRamp(Constants.Climber.RAMP_RATE);
        rightMotor.configClosedloopRamp(Constants.Climber.RAMP_RATE);

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

    }

    /**
     * Release the mechanical stopper of the climber.
     * <p>
     * This would allow it to extend.
     */
    public void releaseStopper() {
        if(safeToClimb())return;
        stopper.set(DoubleSolenoid.Value.kReverse);

    }

    /**
     * Engage the mechanical stopper to lock the climber.
     */
    public void engageStopper() {
        stopper.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Return whether the mechanical stopper is engaged.
     *
     * @return the state of the mechanical stopper
     */
    public boolean isStopperEngaged() {
        return stopper.get().equals(DoubleSolenoid.Value.kForward);
    }

    /**
     * @return the current height of the left side of the climber in meters.
     */
    public double getLeftHeight() {
        return unitModel.toUnits(leftMotor.getSelectedSensorPosition());
    }

    /**
     * Move the left side of the climber to a given height.
     *
     * @param height the height setpoint of the left elevator in meters
     */
    public void setLeftHeight(double height) {
        if (safeToClimb()) {
            leftMotor.set(ControlMode.MotionMagic, unitModel.toTicks(normalizeSetpoint(height)), DemandType.ArbitraryFeedForward, Constants.Climber.ARBITRARY_FEEDFORWARD);
        }
    }

    /**
     * @return the current height of the right side of the climber in meters.
     */
    public double getRightHeight() {
        return unitModel.toUnits(rightMotor.getSelectedSensorPosition());
    }

    /**
     * Move the right side of the climber to a given height.
     *
     * @param height the height setpoint of the right elevator in meters
     */
    public void setRightHeight(double height) {
        if (safeToClimb()) {
            rightMotor.set(ControlMode.MotionMagic, unitModel.toTicks(normalizeSetpoint(height)), DemandType.ArbitraryFeedForward, Constants.Climber.ARBITRARY_FEEDFORWARD);
        }
    }

    /**
     * All cases where we want to prevent the drivers from climbing should return true here. whether it's by game time.
     * It won't allow climbing before the endgame
     *
     * @return whether the robot should not climb
     */
    private boolean safeToClimb() {
        return DriverStation.getInstance().getMatchTime() > 120;
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
     * Reset the encoder position to the maximal height of the left side of subsystem, in case it reaches the limit.
     */
    public void resetLeft() {
        leftMotor.setSelectedSensorPosition(unitModel.toTicks(Constants.Climber.MAX_HEIGHT));
    }

    /**
     * Reset the encoder position to the maximal height of right side of the subsystem, in case it reaches the limit.
     */
    public void resetRight() {
        rightMotor.setSelectedSensorPosition(unitModel.toTicks(Constants.Climber.MAX_HEIGHT));
    }

    /**
     * Return a normalized constrained in a certain range.
     *
     * @param setpoint the setpoint.
     * @return the normalized setpoint.
     */
    public double normalizeSetpoint(double setpoint) {
        if (setpoint > Constants.Climber.MAX_HEIGHT) {
            return Constants.Climber.MAX_HEIGHT;
        } else if (setpoint < 0) {
            return 0;
        }
        return setpoint;
    }

    public double normalizeDelta(double delta){
        if (delta > Constants.Climber.MAX_DIFFERENCE) {
            return Constants.Climber.MAX_DIFFERENCE;
        } else if (delta < -Constants.Climber.MAX_DIFFERENCE) {
            return -Constants.Climber.MAX_DIFFERENCE;
        }
        return delta;
    }

    @Override
    public void periodic() {
        //Reset if the limit switch is pressed.
        if (isLeftOnLimit()) {
            resetLeft();
        }
        if (isRightOnLimit()) {
            resetRight();
        }

        if (isRightOnLimit() || getRightHeight() >= Constants.Climber.MAX_HEIGHT) {
            setRightHeight(Constants.Climber.MAX_HEIGHT);
        }

        if (isLeftOnLimit() || getLeftHeight() >= Constants.Climber.MAX_HEIGHT) {
            setLeftHeight(Constants.Climber.MAX_HEIGHT);
        }

    }
}
