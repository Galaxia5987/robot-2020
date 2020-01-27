package frc.robot.subsystems.drivetrain;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.EKF.KalmanFilter;
import frc.robot.subsystems.drivetrain.KalmanLocalization.OdometryInertialObservation;
import frc.robot.subsystems.drivetrain.KalmanLocalization.OdometryInertialProcess;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import static java.lang.Math.abs;

/**
 * Class for localization using differntial drive odometry and inertial sensors. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders, a gyroscope and an accelerometer.
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
    double m_width;     // Robot width - distance between wheel centers
    final double MAX_ANGLE_DELTA_ENCODER_GYRO = 0.01; // allowed deviation in one cycle of between gyro and encoders. A larger value means wheel is slipping


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
                            Pose2d initialPoseMeters, double widthMeters) {
        m_poseMeters = initialPoseMeters;
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();

        observation = new OdometryInertialObservation(widthMeters / 2, widthMeters / 2);
        process = new OdometryInertialProcess();
        filter = new KalmanFilter(process);
        m_prev_time = 0;
        m_width = widthMeters;
    }

    /**
     * Constructs a DifferentialDriveOdometry object with the default pose at the origin.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     */
    public FullLocalization(Rotation2d gyroAngle, double widthMeters) {
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

        process.setState(0, poseMeters.getTranslation().getX());
        process.setState(1, poseMeters.getTranslation().getY());
        process.setState(2, 0);
        process.setState(3, poseMeters.getRotation().minus(gyroAngle).getRadians());
        process.setState(4, 0);

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
     * Updates the robot position on the field using distance measurements from encoders and IMU. This
     * method is more numerically accurate than using velocities to integrate the pose and
     * is also advantageous for teams that are using lower CPR encoders.
     *
     * @param gyroAngle           The angle reported by the gyroscope.
     * @param leftDistanceMeters  The distance traveled by the left encoder.
     * @param rightDistanceMeters The distance traveled by the right encoder.
     * @param acc                 The acceleration along X axis of the robot [m/s^2]
     * @param time                The current time [s]
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, double leftDistanceMeters,
                         double rightDistanceMeters, double acc, double time) {
        double deltaLeftDistance = leftDistanceMeters - m_prevLeftDistance;
        double deltaRightDistance = rightDistanceMeters - m_prevRightDistance;

        m_prevLeftDistance = leftDistanceMeters;
        m_prevRightDistance = rightDistanceMeters;

         var angle = gyroAngle.plus(m_gyroOffset);
        m_previousAngle = angle;


        double dt = time - m_prev_time;
        // Observation object holds the new measurements
        observation.SetMeasurement(gyroAngle.getRadians(), deltaLeftDistance, deltaRightDistance, dt);
        // Acceleration enters the process and not the observation
        process.setAcc(acc);

        // Check if encoders are valid or slipping:
        observation.setEncoderValid(EncoderValid(gyroAngle, deltaLeftDistance, deltaRightDistance));


        // The main estimate step:
        filter.update(time, observation);

        // Pack the results in Pose2d class
        Rotation2d phi = new Rotation2d(filter.model.state_estimate.data[3][0]);
        m_poseMeters = new Pose2d(filter.model.state_estimate.data[0][0],
                filter.model.state_estimate.data[1][0],
                phi);


        m_prev_time = time;
        return m_poseMeters;
    }

    // Detect encoder slipping condition by comparing gyro and encoder readings.
    public boolean EncoderValid(Rotation2d gyroAngle, double deltaLeftDistance,
                                double deltaRightDistance) {
        double delta_angle = gyroAngle.getRadians() - m_previousAngle.getRadians();

        double delta_angle_from_encoder = (deltaLeftDistance - deltaRightDistance) / m_width;
        SmartDashboard.putNumber("delta angle: " , delta_angle );
        SmartDashboard.putNumber("delta angle from encoder: " , delta_angle_from_encoder );
        if (abs(delta_angle - delta_angle_from_encoder) > MAX_ANGLE_DELTA_ENCODER_GYRO) {
            return false;
        } else {
            return true;
        }
    }
}
