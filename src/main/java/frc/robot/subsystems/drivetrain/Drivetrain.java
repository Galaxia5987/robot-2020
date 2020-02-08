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
    private NetworkTable localizationTable = NetworkTableInstance.getDefault().getTable("localization");
    private NetworkTableEntry x = localizationTable.getEntry("x");
    private NetworkTableEntry y = localizationTable.getEntry("y");
    private NetworkTableEntry velocity = localizationTable.getEntry("velocity");
    private NetworkTableEntry theta = localizationTable.getEntry("theta");
    private NetworkTableEntry angularVelocity = localizationTable.getEntry("angular-velocity");
    private NetworkTableEntry accelerationBias = localizationTable.getEntry("acceleration-bias");
    private NetworkTableEntry encoderLeft = localizationTable.getEntry("left-encoder");
    private NetworkTableEntry encoderRight = localizationTable.getEntry("right-encoder");
    private FullLocalization localization;
    private DifferentialDriveOdometry differentialDriveOdometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(navx.getAngle())),new Pose2d(0, 0, new Rotation2d()));

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
        localization = new FullLocalization( new Rotation2d(0),ROBOT_WIDTH);
        localizationTimer.reset();
        localizationTimer.start();
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

    public Pose2d getPose() {
        return localization.getPoseMeters();
    }

    public void setPose(Pose2d pose, Rotation2d rotation) {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        navx.reset();
        localization.resetPosition(pose, rotation, localizationTimer.get());
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
        Pose2d current = localization.update( new Rotation2d( Math.toRadians(navx.getAngle())),
                unitModel.toUnits(leftMaster.getSelectedSensorPosition()),
                unitModel.toUnits(rightMaster.getSelectedSensorPosition()),
                navx.getWorldLinearAccelX()*GRAVITY_ACCELERATION,
                localizationTimer.get()
        );

        differentialDriveOdometry.update(new Rotation2d( Math.toRadians(navx.getAngle())),
                unitModel.toUnits(leftMaster.getSelectedSensorPosition()),
                unitModel.toUnits(rightMaster.getSelectedSensorPosition()));

        SmartDashboard.putNumber(" simple x", differentialDriveOdometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber(" simple y", differentialDriveOdometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber(" simple angle", differentialDriveOdometry.getPoseMeters().getRotation().getRadians());

        x.setDouble(localization.filter.model.state_estimate.data[0][0]);
        y.setDouble(localization.filter.model.state_estimate.data[1][0]);
        velocity.setDouble(localization.filter.model.state_estimate.data[2][0]);
        theta.setDouble(localization.filter.model.state_estimate.data[3][0]);
        angularVelocity.setDouble(localization.filter.model.state_estimate.data[4][0]);
        accelerationBias.setDouble(localization.filter.model.state_estimate.data[5][0]);
        encoderLeft.setDouble(leftMaster.getSelectedSensorPosition());
        encoderRight.setDouble(rightMaster.getSelectedSensorPosition());

        if (getCooldown() > SHIFTER_COOLDOWN)
            resetCooldown();

        FalconDashboard.INSTANCE.setRobotX(current.getTranslation().getX());
        FalconDashboard.INSTANCE.setRobotY(current.getTranslation().getY());
        FalconDashboard.INSTANCE.setRobotHeading(Math.toRadians(navx.getAngle() * (GYRO_INVERTED ? -1 : 1)));
    }

    public enum shiftModes {
        TOGGLE,
        HIGH,
        LOW
    }
}
