package frc.robot.subsystems;


import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

import frc.robot.EKF.KalmanFilter;
import frc.robot.KalmanLocalization.OdometryInertialObservation;
import frc.robot.KalmanLocalization.OdometryInertialProcess;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

        import edu.wpi.first.hal.FRCNetComm.tInstances;
        import edu.wpi.first.hal.FRCNetComm.tResourceType;
        import edu.wpi.first.hal.HAL;
        import edu.wpi.first.wpilibj.geometry.Pose2d;
        import edu.wpi.first.wpilibj.geometry.Rotation2d;
        import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Class for differential drive odometry. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders and a gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 *
 * <p>It is important that you reset your encoders to zero before using this class.
 * Any subsequent pose resets also require the encoders to be reset to zero.
 */
public class FullLocalization {

    public KalmanFilter filter;
    private OdometryInertialProcess process;
    private OdometryInertialObservation observation;
    double time = 0;


    private Pose2d m_poseMeters;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;

    private double m_prevLeftDistance;
    private double m_prevRightDistance;
    private double m_prev_time;

    /**
     * Constructs a DifferentialDriveOdometry object.
     *
     * @param gyroAngle         The angle reported by the gyroscope.
     * @param initialPoseMeters The starting position of the robot on the field.
     */
    public FullLocalization(Rotation2d gyroAngle,
                                     Pose2d initialPoseMeters,double widthMeters) {
        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();

        observation =  new OdometryInertialObservation(widthMeters/2,widthMeters/2);
        process = new OdometryInertialProcess();
        filter = new KalmanFilter(process);
        m_prev_time = 0;
    }

    /**
     * Constructs a DifferentialDriveOdometry object with the default pose at the origin.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public FullLocalization(Rotation2d gyroAngle,double widthMeters) {
        this(gyroAngle, new Pose2d(), widthMeters);
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>You NEED to reset your encoders (to zero) when calling this method.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code.
     * The library automatically takes care of offsetting the gyro angle.
     *
     * @param poseMeters The position on the field that your robot is at.
     * @param gyroAngle  The angle reported by the gyroscope.
     */
    public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle, double time) {
        m_poseMeters = poseMeters;
        m_previousAngle = poseMeters.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);

        m_prevLeftDistance = 0.0;
        m_prevRightDistance = 0.0;

        m_prev_time = time;

        process.setState(0,poseMeters.getTranslation().getX());
        process.setState(1,poseMeters.getTranslation().getY());
        process.setState(2,0);
        process.setState(3,poseMeters.getRotation().minus(gyroAngle).getRadians());
        process.setState(4,0);

    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }


    /**
     * Updates the robot position on the field using distance measurements from encoders. This
     * method is more numerically accurate than using velocities to integrate the pose and
     * is also advantageous for teams that are using lower CPR encoders.
     *
     * @param gyroAngle           The angle reported by the gyroscope.
     * @param leftDistanceMeters  The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, double leftDistanceMeters,
                         double rightDistanceMeters, double acc, double time) {
        double deltaLeftDistance = leftDistanceMeters - m_prevLeftDistance;
        double deltaRightDistance = rightDistanceMeters - m_prevRightDistance;

        m_prevLeftDistance = leftDistanceMeters;
        m_prevRightDistance = rightDistanceMeters;

        double averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;
        var angle = gyroAngle.plus(m_gyroOffset);

        var newPose = m_poseMeters.exp(
                new Twist2d(averageDeltaDistance, 0.0, angle.minus(m_previousAngle).getRadians()));

        m_previousAngle = angle;


        double dt = time-m_prev_time;
        observation.SetMeasurement(gyroAngle.getRadians(),deltaLeftDistance,deltaRightDistance,dt);
        process.setAcc(acc);

        filter.update(time,observation);

        Rotation2d phi = new Rotation2d( filter.model.state_estimate.data[3][0]);
        m_poseMeters = new Pose2d(filter.model.state_estimate.data[0][0],
                filter.model.state_estimate.data[1][0],
                phi);



        m_prev_time = time;
        return m_poseMeters;
    }
}
