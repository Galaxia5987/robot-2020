package frc.robot.subsystems.drivetrain.KalmanLocalization;

import frc.robot.subsystems.drivetrain.EKF.ObservationModel;

// Class that implements the observation model for localization Kalman filter
public class OdometryInertialObservation  extends ObservationModel {

    // Measurements:
    private double m_yaw;
    private double m_LeftTrackSpeed;
    private double m_RightTrackSpeed;

    private double m_Rr; // distance from robot IMU to right wheel
    private double m_Rl; // distacne from robot IMU to left wheel

    private boolean m_encoderValid = true; // Encoder valid flag

    public void setEncoderValid(boolean valid){
        m_encoderValid = valid;
    }

    // Constructor must receive the geometry of the robot
    public OdometryInertialObservation(double Rr, double Rl)
    {
        m_Rl = Rl;
        m_Rr = Rr;
    }

    public void SetMeasurement(double yaw, double LeftTrack, double RightTrack, double dt){
        m_yaw = yaw;
        m_LeftTrackSpeed = LeftTrack/dt;
        m_RightTrackSpeed = RightTrack/dt;
    }


    @Override
    public int observationDimension() {
        return 3;
    }

    @Override
    public int stateDimension() {
        return 6;
    }

    @Override
    public void observationMeasurement(double[][] y) {
        y[0][0] = m_yaw;
        y[1][0] = m_LeftTrackSpeed;
        y[2][0] = m_RightTrackSpeed;
    }

    @Override
    public void observationModel(double[][] x, double[][] h) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];

        h[0][0] = phi; // First measurement is the gyro angle
        h[1][0] = v + m_Rl*omega; // Second measurement is the left encoder
        h[2][0] = v - m_Rr*omega; // Third measurement is the right encoder
    }

    @Override
    public void observationModelJacobian(double[][] x, double[][] j) {
        j[0][3] = 1;
        j[1][2] = 1;
        j[2][2] = 1;
        j[1][4] = m_Rl;
        j[2][4] = -m_Rr; // TODO: check if the direction of encoders is correct in Kalman tuning

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

    }

}