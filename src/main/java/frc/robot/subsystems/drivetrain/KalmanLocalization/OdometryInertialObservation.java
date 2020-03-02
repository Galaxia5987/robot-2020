package frc.robot.subsystems.drivetrain.KalmanLocalization;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.drivetrain.EKF.ObservationModel;
import org.opencv.core.Mat;

// Class that implements the observation model for localization Kalman filter
public class OdometryInertialObservation  extends ObservationModel {

    // Measurements:
    private double m_yaw;
    private double m_LeftTrackSpeed;
    private double m_RightTrackSpeed;

    private double m_Rr; // distance from robot IMU to right wheel
    private double m_Rl; // distacne from robot IMU to left wheel

    private boolean m_encoderValid = true; // Encoder valid flag
    private double m_Range;
    private double m_TargetAngle;
    private Pose2d m_TargetPose;
    private boolean m_targetValid;

    private double m_ExpectedRangeSqr;

    public void setEncoderValid(boolean valid){
        m_encoderValid = valid;
    }

    public void setTargetValid(boolean valid) { m_targetValid = valid;}

    // Constructor must receive the geometry of the robot
    public OdometryInertialObservation(double Rr, double Rl)
    {
        m_Rl = Rl;
        m_Rr = Rr;
    }

    /**
     *
     * @param yaw - body angles in radians
     * @param LeftTrack
     * @param RightTrack
     * @param Range
     * @param TargetAngle - view angle of target in radians.
     * @param TargetPose
     * @param dt
     */
    public void SetMeasurement(double yaw, double LeftTrack, double RightTrack, double Range, double TargetAngle,
                               Pose2d TargetPose, double dt){
        m_yaw = yaw;
        m_LeftTrackSpeed = LeftTrack/dt;
        m_RightTrackSpeed = RightTrack/dt;
        m_Range = Range;
        m_TargetAngle = TargetAngle;
        m_TargetPose = TargetPose;
        
    }


    @Override
    public int observationDimension() {
        return 5;
    }

    @Override
    public int stateDimension() {
        return 7;
    }

    @Override
    public void observationMeasurement(double[][] y) {
        y[0][0] = m_yaw;
        y[1][0] = m_LeftTrackSpeed;
        y[2][0] = m_RightTrackSpeed;
        y[3][0] = m_Range*m_Range;
        y[4][0] = m_TargetAngle;
    }

    @Override
    public void observationModel(double[][] x, double[][] h) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];

        double dx = m_TargetPose.getTranslation().getX() - X;
        double dy = m_TargetPose.getTranslation().getY() - Y;


        h[0][0] = phi; // First measurement is the gyro angle
        h[1][0] = v + m_Rl*omega; // Second measurement is the left encoder
        h[2][0] = v - m_Rr*omega; // Third measurement is the right encoder

        h[3][0] = dx*dx + dy*dy;
        h[4][0] = Math.IEEEremainder( Math.atan2(dy,dx) - phi,2*Math.PI);

        m_ExpectedRangeSqr = h[3][0];
    }

    public double GetExpectedRange()
    {
        return Math.sqrt(m_ExpectedRangeSqr);
    }
    @Override
    public void observationModelJacobian(double[][] x, double[][] j) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];

        double dx = m_TargetPose.getTranslation().getX() - X;
        double dy = m_TargetPose.getTranslation().getY() - Y;

        double r_sqr = dx*dx + dy*dy;


        j[0][3] = 1;
        j[1][2] = 1;
        j[2][2] = 1;
        j[1][4] = m_Rl;
        j[2][4] = -m_Rr; // TODO: check if the direction of encoders is correct in Kalman tuning

        j[3][0] = -2*dx;
        j[3][1] = -2*dy;

        j[4][0] = dy/r_sqr;
        j[4][1] = -dx/r_sqr;

        j[4][3] = -1;
    }

    @Override
    public void observationNoiseCovariance(double[][] cov) {
        cov[0][0] = 1e-8;     // sigma = 1e-4 rad

        if (m_encoderValid) {
            cov[1][1] = 1e-6; // sigma = 1e-3 m/s
            cov[2][2] = 1e-6; // sigma = 1e-3 m/s
        } else{ // Throw away encoders if it slips
            cov[1][1] = 1e6; // garbage measurement
            cov[2][2] = 1e6; // garbage measurement
            System.out.println("encoders invalid");
        }


        if (m_targetValid)
        {
            // TODO: tune vision and turret errors
            cov[3][3] = 1 ; // 10 cm range error for 5 m range (5+0.1)^2 = 25 + 2*0.5 + 0.01; => std = 1 => cov = 1
            cov[4][4] = 1e6 ; // about 0.5 degrees 1e-4
        }
        else
        {
            cov[3][3] = 1e6; // garbage measurement
            cov[4][4] = 1e6; // garbage measurement

        }

    }

}