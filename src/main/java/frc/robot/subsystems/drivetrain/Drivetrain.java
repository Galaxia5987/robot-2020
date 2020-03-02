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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.CustomDashboard;
import frc.robot.utilities.FalconConfiguration;
import frc.robot.utilities.Utils;
import frc.robot.utilities.VisionModule;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.ghrobotics.lib.debug.FalconDashboard;
import org.techfire225.webapp.FireLog;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.EFFECTIVE_TURN_WIDTH;
import static frc.robot.Constants.FieldGeometry.FIELD_WIDTH;
import static frc.robot.Ports.Drivetrain.LEFT_MASTER_INVERTED;
import static frc.robot.Ports.Drivetrain.LEFT_SLAVE_INVERTED;
import static frc.robot.Ports.Drivetrain.RIGHT_MASTER_INVERTED;
import static frc.robot.Ports.Drivetrain.RIGHT_SLAVE_INVERTED;
import static frc.robot.Ports.Drivetrain.*;
import static frc.robot.RobotContainer.navx;

public class Drivetrain extends SubsystemBase {
    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    private UnitModel lowGearUnitModel = new UnitModel(LOW_TICKS_PER_METER);
    private UnitModel highGearUnitModel = new UnitModel(HIGH_TICKS_PER_METER);
    public static final FullLocalization localization = new FullLocalization(new Rotation2d(0), EFFECTIVE_TURN_WIDTH);

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

        new WebConstantPIDTalon("drivetrainLeft", KP, KI, KD, KF, leftMaster);
        new WebConstantPIDTalon("drivetrainRight", KP, KI, KD, KF, rightMaster);

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
        motorConfigurations.setPidSet(KP, KI, KD, KF);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setEnableCurrentLimit(true);
        motorConfigurations.setSupplyCurrentLimit(40);
        UtilityFunctions.configAllFalcons(motorConfigurations, rightMaster, rightSlave, leftMaster, leftSlave);
        if (Robot.isRobotA)
            gearShifterA = new DoubleSolenoid(SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
        else
            gearShifterB = new Solenoid(SHIFTER_PORT);

        navx.reset();
 //       Pose2d INITIAL_POSE = new Pose2d(UtilityFunctions.getAlliancePort(false).getTranslation().getX() - 10, UtilityFunctions.getAlliancePort(false).getTranslation().getY(), new Rotation2d());
        Pose2d INITIAL_POSE = new Pose2d(15.98 - 3.2, 8.23 - 2.42, new Rotation2d(Math.PI));
        localization.resetPosition(INITIAL_POSE, new Rotation2d(Math.toRadians(navx.getAngle())), Robot.robotTimer.get());
        odometry.resetPosition(INITIAL_POSE, Rotation2d.fromDegrees(getHeading()));
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
                && !isShiftedHigh();
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
                && Math.abs(getRightVelocity()) < SHIFT_SPEED_TOLERANCE; //Shifting low at high speeds can cause damage to the motors.

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

    public void setBrake(boolean brake) {
        NeutralMode neutralMode = brake ? NeutralMode.Brake : NeutralMode.Coast;
        leftMaster.setNeutralMode(neutralMode);
        leftSlave.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
    }

    /**
     * @return the velocity of the left motor
     */
    public double getLeftVelocity() {
        return getCurrentUnitModel().toVelocity(leftMaster.getSelectedSensorVelocity());
    }

    public double getLeftPosition() {
        return getCurrentUnitModel().toUnits(leftMaster.getSelectedSensorPosition());
    }

    public double getRightPosition() {
        return getCurrentUnitModel().toUnits(rightMaster.getSelectedSensorPosition());
    }

    /**
     * Indicates whether the shifter is on a high gear
     *
     * @return
     */
    public boolean isShiftedHigh() {
        if (Robot.isRobotA && gearShifterA != null)
            return gearShifterA.get() == DoubleSolenoid.Value.kForward || gearShifterA.get() == DoubleSolenoid.Value.kOff;
        else if (gearShifterB != null)
            return !gearShifterB.get();
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

    public double getCCWHeading() {
        return -getHeading();
    }

    public Pose2d getPose() {
        return flipCoordSystem( localization.getPoseMeters());
    }

    public void setPose(Pose2d pose) {
        pose = flipCoordSystem(pose);
        Rotation2d rotation =  new Rotation2d(Math.toRadians(navx.getAngle()));
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        navx.reset();
        odometry.resetPosition(pose, rotation);
        localization.resetPosition(pose, rotation, Robot.robotTimer.get());
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

    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        if (getCooldown() > SHIFTER_COOLDOWN)
            resetCooldown();

        SmartDashboard.putBoolean("shiftedHigh", isShiftedHigh());
        SmartDashboard.putNumber("leftPosition", getLeftPosition());
        SmartDashboard.putNumber("rightPosition", getRightPosition());

        Pose2d current = localization.update( new Rotation2d( Math.toRadians(navx.getAngle())),
                getLeftPosition(),
                getRightPosition(),
                navx.getWorldLinearAccelY()*GRAVITY_ACCELERATION,
                Robot.robotTimer.get()
        );

        odometry.update(new Rotation2d( Math.toRadians(navx.getAngle())),
                getLeftPosition(),
                getRightPosition());

        SmartDashboard.putNumber(" simple x", odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber(" simple y", odometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber(" simple angle", odometry.getPoseMeters().getRotation().getRadians());
        SmartDashboard.putNumber("navx accel", navx.getWorldLinearAccelY() * GRAVITY_ACCELERATION);

        Pose2d falcon_pose = getPose();
        FalconDashboard.INSTANCE.setRobotX(Units.metersToFeet(falcon_pose.getTranslation().getX()));
        FalconDashboard.INSTANCE.setRobotY(Units.metersToFeet(falcon_pose.getTranslation().getY()));
        FalconDashboard.INSTANCE.setRobotHeading(falcon_pose.getRotation().getRadians());
    }

    /**
     * shiftModes.HIGH is the default on the robot, and means high speed.
     * shiftModes.LOW is for low speed, but more power.
     */
    public enum shiftModes {
        HIGH,
        LOW
    }

    /**
     *  Flips coordinate definition from Falcon (0,0) bottom left to Galaxia (0,0) top left and the reverse
     * @param pose
     * @return
     */
    public Pose2d flipCoordSystem(Pose2d pose )
    {
        return new Pose2d(pose.getTranslation().getX(),FIELD_WIDTH -  pose.getTranslation().getY(), new Rotation2d(- pose.getRotation().getRadians()) );
    }
}
