/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.FalconConfiguration;

import static frc.robot.Ports.Drivetrain.LEFT_MASTER;
import static frc.robot.Ports.Drivetrain.LEFT_SLAVE;
import static frc.robot.Ports.Drivetrain.RIGHT_MASTER;
import static frc.robot.Ports.Drivetrain.RIGHT_SLAVE;
import static frc.robot.Ports.Drivetrain.SHIFTER_FORWARD_PORT;
import static frc.robot.Ports.Drivetrain.SHIFTER_PORT;
import static frc.robot.Ports.Drivetrain.SHIFTER_REVERSE_PORT;

public class Drivetrain extends SubsystemBase {


    private final TalonFX rightMaster = new TalonFX(RIGHT_MASTER);
    private final TalonFX rightSlave = new TalonFX(RIGHT_SLAVE);
    private final TalonFX leftMaster = new TalonFX(LEFT_MASTER);
    private final TalonFX leftSlave = new TalonFX(LEFT_SLAVE);
    private FalconConfiguration configurations = new FalconConfiguration();
    private double[] pidSet = {Constants.Drivetrain.KP, Constants.Drivetrain.KI, Constants.Drivetrain.KD, Constants.Drivetrain.KF};

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    private AHRS navx = new AHRS(SPI.Port.kMXP);
    private final UnitModel unitModel = new UnitModel(Constants.Drivetrain.TICKS_PER_METER);

    private DoubleSolenoid AgearShifter = new DoubleSolenoid(1, SHIFTER_FORWARD_PORT, SHIFTER_REVERSE_PORT);
    private Solenoid BgearShifter = new Solenoid(1, SHIFTER_PORT);

    /**
     * Creates a new ExampleSubsystem.
     */
    public Drivetrain() {
        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);
        configurations.setNeutralMode(NeutralMode.Coast);
        configurations.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        configurations.setEnableVoltageCompensation(true);
        configurations.setPidSet(pidSet);
        configurations.setEnableCurrentLimit(true);
        configurations.setEnableCurrentLimit(true);
        configurations.setSupplyCurrentLimit(40);
        UtilityFunctions.configAllFalcons(configurations, rightMaster, rightSlave, leftMaster, leftSlave);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    public void initialize() {
        navx.reset();
        odometry.resetPosition(new Pose2d(), new Rotation2d()); //Set to actual starting position
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                unitModel.toUnits(leftMaster.getSelectedSensorPosition()),
                unitModel.toUnits(rightMaster.getSelectedSensorPosition())
        );
    }
}
