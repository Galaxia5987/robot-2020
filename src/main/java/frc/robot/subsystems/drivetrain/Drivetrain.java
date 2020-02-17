/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.FalconConfiguration;
import frc.robot.utilities.Utils;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.ghrobotics.lib.debug.FalconDashboard;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Ports.Drivetrain.*;
import static frc.robot.RobotContainer.navx;

public class Drivetrain extends SubsystemBase {
    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    private double[] pidSet = {VELOCITY_PID_SET[0], VELOCITY_PID_SET[1], VELOCITY_PID_SET[2], VELOCITY_PID_SET[3]};
    private UnitModel lowGearUnitModel = new UnitModel(LOW_TICKS_PER_METER);
    private UnitModel highGearUnitModel = new UnitModel(HIGH_TICKS_PER_METER);
    /**
     * The gear shifter will be programmed according to the following terms
     * High gear - low torque High speed
     * Low gear - high torque Low speed
     */
    private DoubleSolenoid gearShifterA = null;
    private Solenoid gearShifterB = null;
    private Timer shiftCooldown = new Timer();
    private boolean isShifting = false;


    public Drivetrain() {
        FalconConfiguration motorConfigurations = new FalconConfiguration();

        new WebConstantPIDTalon("drivetrainLeft", pidSet[0], pidSet[1], pidSet[2], pidSet[3], leftMaster);
        new WebConstantPIDTalon("drivetrainRight", pidSet[0], pidSet[1], pidSet[2], pidSet[3], rightMaster);

        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();
        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();

        rightMaster.setSelectedSensorPosition(0);
        leftMaster.setSelectedSensorPosition(0);
        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);

        //Inversions
        rightMaster.setInverted(RIGHT_MASTER_INVERTED);
        rightSlave.setInverted(RIGHT_SLAVE_INVERTED);
        leftMaster.setInverted(LEFT_MASTER_INVERTED);
        leftSlave.setInverted(LEFT_SLAVE_INVERTED);

        motorConfigurations.setNeutralMode(NeutralMode.Coast);
        motorConfigurations.setEnableVoltageCompensation(true);
        motorConfigurations.configureVoltageCompensationSaturation(12);
        motorConfigurations.setPidSet(pidSet[0], pidSet[1], pidSet[2], pidSet[3]);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setSupplyCurrentLimit(40);
        Utils.configAllFalcons(motorConfigurations, rightMaster, rightSlave, leftMaster, leftSlave);
        if (Robot.isRobotA)
            gearShifterA = new DoubleSolenoid(SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
        else
            gearShifterB = new Solenoid(SHIFTER_PORT);
    }

    public void shiftGear(shiftModes mode) {
        switch (mode) {
            case LOW:
                if (canShiftLow())
                    shiftLow();
                break;
            case HIGH:
                if (canShiftHigh())
                    shiftHigh();
                break;
        }
    }

    /**
     * Start the cooldown of the shifter so it won't shift too open
     */
    public void startCooldown() {
        shiftCooldown.start();
        isShifting = true;
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
            gearShifterB.set(!IS_SHIFTER_REVERSED);
    }

    private void shiftLow() {
        startCooldown();
        if (Robot.isRobotA)
            gearShifterA.set(DoubleSolenoid.Value.kReverse);
        else
            gearShifterB.set(IS_SHIFTER_REVERSED);
    }

    /**
     * Checks if the drivetrain is  able to switch to high gear
     *
     * @return if the drivetrain can shift to high gear
     */
    private boolean canShiftHigh() {
        return !isShifting
                && !isShiftedHigh()
                && Math.abs(getLeftVelocity()) < SHIFT_SPEED_TOLERANCE
                && Math.abs(getRightVelocity()) < SHIFT_SPEED_TOLERANCE;
    }

    /**
     * Checks if the drivetrain is  able to switch to low gear
     *
     * @return if the drivetrain can shift to low gear
     */
    private boolean canShiftLow() {
        return !isShifting
                && !isShiftedLow()
                && Math.abs(getLeftVelocity()) < SHIFT_SPEED_TOLERANCE
                && Math.abs(getRightVelocity()) < SHIFT_SPEED_TOLERANCE;

    }

    public UnitModel getCurrentUnitModel() {
        return isShiftedHigh() ? highGearUnitModel : lowGearUnitModel;
    }

    /**
     * @return the velocity of the right motor
     */
    public double getRightVelocity() {
        return getCurrentUnitModel().toVelocity(rightMaster.getSelectedSensorVelocity());
    }

    /**
     * @return the velocity of the left motor
     */
    public double getLeftVelocity() {
        return getCurrentUnitModel().toVelocity(leftMaster.getSelectedSensorVelocity());
    }

    /**
     * Indicates whether the shifter is on a high gear
     *
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
     *
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
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose, Rotation2d rotation) {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        navx.reset();
        odometry.resetPosition(pose, rotation);
    }

    public void setVelocityAndFeedForward(double leftVelocity, double rightVelocity, double leftFF, double rightFF) {
        UnitModel unitModel = getCurrentUnitModel();
        leftMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(leftVelocity), DemandType.ArbitraryFeedForward, leftFF);
        rightMaster.set(ControlMode.Velocity, unitModel.toTicks100ms(rightVelocity), DemandType.ArbitraryFeedForward, rightFF);
    }

    public void setPower(double leftPower, double rightPower) {
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        UnitModel unitModel = getCurrentUnitModel();
        Pose2d current = odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                unitModel.toUnits(leftMaster.getSelectedSensorPosition()),
                unitModel.toUnits(rightMaster.getSelectedSensorPosition())
        );
        if (getCooldown() > SHIFTER_COOLDOWN)
            resetCooldown();

        FalconDashboard.INSTANCE.setRobotX(current.getTranslation().getX());
        FalconDashboard.INSTANCE.setRobotY(current.getTranslation().getY());
        FalconDashboard.INSTANCE.setRobotHeading(Math.toRadians(navx.getAngle()));

        SmartDashboard.putBoolean("shiftedHigh", isShiftedHigh());
    }

    /**
     * shiftModes.HIGH is the default on the robot, and means high speed.
     * shiftModes.LOW is for low speed, but more power.
     */
    public enum shiftModes {
        HIGH,
        LOW
    }
}
