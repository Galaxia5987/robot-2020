package frc.robot.KalmanLocalization;

import frc.robot.EKF.ObservationModel;

public class OdometryInertialObservation  extends ObservationModel {

    private double m_yaw;
    private double m_LeftTrackSpeed;
    private double m_RightTrackSpeed;

    private double m_Rr;
    private double m_Rl;

    private boolean m_encoderValid = true;

    public void setEncoderValid(boolean valid){
        m_encoderValid = valid;
    }

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
        return 5;
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

        h[0][0] = phi;
        h[1][0] = v - m_Rl*omega;
        h[2][0] = v + m_Rr*omega;
    }

    @Override
    public void observationModelJacobian(double[][] x, double[][] j) {
        j[0][3] = 1;
        j[1][2] = 1;
        j[2][2] = 1;
        j[1][4] = -m_Rl;
        j[2][4] = m_Rr;

    }

    @Override
    public void observationNoiseCovariance(double[][] cov) {
        cov[0][0] = 4e-4;     // sigma = 2e-2 rad

        if (m_encoderValid) {
            cov[1][1] = 1e-4; // sigma = 1e-2 m/s
            cov[2][2] = 1e-4; // sigma = 1e-2 m/s
        }
        else{
            cov[1][1] = 1e6; // garbage measurement
            cov[2][2] = 1e6; // garbage measurement
        }

    }

}