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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.TalonConfiguration;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.Climber.CLIMB_PIDF;
import static frc.robot.Constants.Climber.CLIMB_RELEASE_PIDF;
import static frc.robot.Ports.Climber.IS_STOPPER_REVERSED;

public class Climber extends SubsystemBase {

    private final TalonSRX leftMotor = new TalonSRX(Ports.Climber.LEFT_MOTOR);
    private final TalonSRX rightMotor = new TalonSRX(Ports.Climber.RIGHT_MOTOR);
    private final UnitModel unitModel = new UnitModel(Constants.Climber.TICKS_PER_METER);
    private DoubleSolenoid stopperA = null;
    private Solenoid stopperB = null;

    /**
     * Creates a new climb Subsystem.
     */
    public Climber() {
        TalonConfiguration talonConfigs = new TalonConfiguration();
        leftMotor.setInverted(Ports.Climber.LEFT_MOTOR_INVERTED);
        rightMotor.setInverted(Ports.Climber.RIGHT_MOTOR_INVERTED);

        leftMotor.setSensorPhase(Ports.Climber.LEFT_ENCODER_INVERTED);
        rightMotor.setSensorPhase(Ports.Climber.RIGHT_ENCODER_INVERTED);

        leftMotor.configClosedloopRamp(Constants.Climber.RAMP_RATE);
        rightMotor.configClosedloopRamp(Constants.Climber.RAMP_RATE);
      
        talonConfigs.setPidSet(CLIMB_PIDF[0], CLIMB_PIDF[1], CLIMB_PIDF[2], CLIMB_PIDF[3]);
        talonConfigs.setForwardLimitSwitchSource(LimitSwitchSource.Deactivated);
        talonConfigs.setForwardLimitSwitchNormal(LimitSwitchNormal.Disabled);
        talonConfigs.setReverseLimitSwitchSource(LimitSwitchSource.Deactivated);
        talonConfigs.setReverseLimitSwitchNormal(LimitSwitchNormal.Disabled);

        talonConfigs.setNeutralMode(NeutralMode.Coast);
        talonConfigs.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Relative);

        talonConfigs.setPeakCurrentLimit(0);
        talonConfigs.setContinuousCurrentLimit(35);
        talonConfigs.setEnableCurrentLimit(true);
        UtilityFunctions.configAllTalons(talonConfigs, leftMotor, rightMotor);

        leftMotor.config_kP(0, CLIMB_PIDF[0]);
        leftMotor.config_kI(0, CLIMB_PIDF[1]);
        leftMotor.config_kD(0, CLIMB_PIDF[2]);
        leftMotor.config_kF(0, CLIMB_PIDF[3]);
        rightMotor.config_kP(0, CLIMB_PIDF[0]);
        rightMotor.config_kI(0, CLIMB_PIDF[1]);
        rightMotor.config_kD(0, CLIMB_PIDF[2]);
        rightMotor.config_kF(0, CLIMB_PIDF[3]);

        leftMotor.config_kP(1, CLIMB_RELEASE_PIDF[0]);
        leftMotor.config_kI(1, CLIMB_RELEASE_PIDF[1]);
        leftMotor.config_kD(1, CLIMB_RELEASE_PIDF[2]);
        leftMotor.config_kF(1, CLIMB_RELEASE_PIDF[3]);
        rightMotor.config_kP(1, CLIMB_RELEASE_PIDF[0]);
        rightMotor.config_kI(1, CLIMB_RELEASE_PIDF[1]);
        rightMotor.config_kD(1, CLIMB_RELEASE_PIDF[2]);
        rightMotor.config_kF(1, CLIMB_RELEASE_PIDF[3]);

        new WebConstantPIDTalon("climbLeft", CLIMB_RELEASE_PIDF[0],  CLIMB_RELEASE_PIDF[1],  CLIMB_RELEASE_PIDF[2],  CLIMB_RELEASE_PIDF[3], leftMotor);
        new WebConstantPIDTalon("climbRight", CLIMB_RELEASE_PIDF[0],  CLIMB_RELEASE_PIDF[1],  CLIMB_RELEASE_PIDF[2],  CLIMB_RELEASE_PIDF[3], rightMotor);

        if (Robot.isRobotA)
            stopperA = new DoubleSolenoid(Ports.Climber.STOPPER_FORWARD, Ports.Climber.STOPPER_REVERSE);
        else
            stopperB = new Solenoid(Ports.Climber.STOPPER);
    }

    /**
     * Release the mechanical stopper of the climber.
     * <p>
     * This would allow it to extend.
     */
    public void releaseStopper() {
        if (safeToClimb()) {
            if (Robot.isRobotA)
                stopperA.set(DoubleSolenoid.Value.kReverse);
            else
                stopperB.set(IS_STOPPER_REVERSED);
        }
    }

    /**
     * Engage the mechanical stopper to lock the climber.
     */
    public void engageStopper() {
        if (Robot.isRobotA)
            stopperA.set(DoubleSolenoid.Value.kForward);
        else
            stopperB.set(!IS_STOPPER_REVERSED);
    }

    /**
     * Return whether the mechanical stopper is engaged.
     *
     * @return the state of the mechanical stopper
     */
    public boolean isStopperEngaged() {
        if (Robot.isRobotA)
            return stopperA.get().equals(DoubleSolenoid.Value.kForward);
        return stopperB.get() != IS_STOPPER_REVERSED;
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
            leftMotor.set(ControlMode.Position, unitModel.toTicks(normalizeSetpoint(height)), DemandType.ArbitraryFeedForward, Constants.Climber.ARBITRARY_FEEDFORWARD);
        }
    }

    public void setLeftPower(double power) {
        leftMotor.set(ControlMode.PercentOutput, power);
    }

    public void setRightPower(double power) {
        rightMotor.set(ControlMode.PercentOutput, power);
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
            rightMotor.set(ControlMode.Position, unitModel.toTicks(normalizeSetpoint(height)), DemandType.ArbitraryFeedForward, Constants.Climber.ARBITRARY_FEEDFORWARD);
        }
    }

    /**
     * All cases where we want to prevent the drivers from climbing should return true here. whether it's by game time.
     * It won't allow climbing before the endgame
     *
     * @return whether the robot should not climb
     */
    private boolean safeToClimb() {
        return Robot.debug || DriverStation.getInstance().getMatchTime() > 120;
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

    public void changePIDFSlot(int slot) {
        rightMotor.selectProfileSlot(slot, 0);
        leftMotor.selectProfileSlot(slot, 0);
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

    public double normalizeDelta(double delta) {
        if (delta > Constants.Climber.MAX_DIFFERENCE) {
            return Constants.Climber.MAX_DIFFERENCE;
        } else if (delta < -Constants.Climber.MAX_DIFFERENCE) {
            return -Constants.Climber.MAX_DIFFERENCE;
        }
        return delta;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climbLeftHeight", getLeftHeight());
        SmartDashboard.putNumber("climbRightHeight", getRightHeight());

        if (getRightHeight() >= Constants.Climber.MAX_HEIGHT) {
            setRightHeight(Constants.Climber.MAX_HEIGHT);
        }

        if (getLeftHeight() >= Constants.Climber.MAX_HEIGHT) {
            setLeftHeight(Constants.Climber.MAX_HEIGHT);
        }

    }
}
