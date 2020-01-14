package frc.robot.KalmanLocalization;
import frc.robot.EKF.ProcessModel;

import static java.lang.Math.cos;
import static java.lang.Math.sin;


// State [x y v phi omega]
//      [ 0 1 2  3   4  ]
//       x [m] , y [m], v [m/s], phi [rad], omega [rad/s]
public class OdometryInertialProcess extends ProcessModel {

    private double m_acc = 0;

    public void setAcc(double acc)
    {
        m_acc = acc;
    }

    @Override
    public int stateDimension() {
        return 5;
    }

    @Override
    public void initialState(double[][] x) {
        x[0][0] = 0; // initial position
        x[1][0] = 0; // initial position
        x[2][0] = 0;
        x[3][0] = 0;
        x[4][0] = 0;

    }

    @Override
    public void initialStateCovariance(double[][] cov) {
        cov[0][0] = 0.01;
        cov[1][1] = 0.01;
        cov[2][2] = 0.1;
        cov[3][3] = 8e-3; //  5 deg sqrd in rad
        cov[4][4] = 1e-5; //  assume not moving
    }

    @Override
    public void stateFunction(double[][] x, double[][] f) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];

        f[0][0] = v * cos(phi);
        f[1][0] = v * sin(phi);
        f[2][0] = m_acc;
        f[3][0] = omega;
        f[4][0] = 0;
    }

    @Override
    public void stateFunctionJacobian(double[][] x, double[][] j) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];

        j[0][2] = cos(phi);
        j[1][2] = sin(phi);
        j[0][3] = -v * sin(phi);
        j[1][3] = v * cos(phi);
        j[3][4] = 1;

    }

    @Override
    public void processNoiseCovariance(double[][] cov) {
        cov[0][0] = 1e-9;
        cov[1][1] = 1e-9;
        cov[2][2] = 1e-2;
        cov[3][3] = 1e-9;
        cov[4][4] = 1e-2;
    }

}