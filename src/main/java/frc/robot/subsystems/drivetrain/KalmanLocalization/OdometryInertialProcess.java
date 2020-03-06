package frc.robot.subsystems.drivetrain.KalmanLocalization;
import frc.robot.subsystems.drivetrain.EKF.ProcessModel;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

// Class that describes the 2D dynamics of robot. Has 5 states as following
// State [x y v phi omega acc_bias]
//      [ 0 1 2  3   4  5]
//       x [m] , y [m], v [m/s], phi [rad], omega [rad/s] acc_bias[m/s^2]
public class OdometryInertialProcess extends ProcessModel {

    private double m_acc = 0;

    public void setAcc(double acc)
    {
        m_acc = acc;
    }

    @Override
    public int stateDimension() {
        return 7;
    }

    @Override
    public void initialState(double[][] x) {
        x[0][0] = 0; // initial position
        x[1][0] = 0; // initial position
        x[2][0] = 0;
        x[3][0] = 0;
        x[4][0] = 0;
        x[5][0] = 0;
        x[6][0] = 0;


    }

    @Override
    public void initialStateCovariance(double[][] cov) {
        // Sets initial variance for state variables.
        cov[0][0] = 0.01; // 10 cm accuracy for X
        cov[1][1] = 0.1; // 10 cm accuracy for Y
        cov[2][2] = 0.0001;  // 0.01 m/s accuracy for X
        cov[3][3] = 8e-3; //  5 deg sqrd in rad for phi
        cov[4][4] = 1e-5; //  assume not moving : 0.2 deg/s for omega
        cov[5][5] = 1e0; //  assume 1 m/s^2
        cov[6][6] = 100; //  assume 10 rad
    }

    @Override
    public void stateFunction(double[][] x, double[][] f) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];
        double acc_bias = x[5][0];
        double angle_bias = x[6][0];

        // The main system dynamics:
        f[0][0] = v * cos(phi); // 2 D motion
        f[1][0] = v * sin(phi); // 2 D motion
        f[2][0] = m_acc-acc_bias;  // Acceleration enters here
        f[3][0] = omega;        // phi derivative is omega
        f[4][0] = 0;            // Assume constant omega
        f[5][0] = 0;            // Assume constant bias
        f[6][0] = 0;            // Assume constant bias
    }

    @Override
    public void stateFunctionJacobian(double[][] x, double[][] j) {
        double X = x[0][0];
        double Y = x[1][0];
        double v = x[2][0];
        double phi = x[3][0];
        double omega = x[4][0];
        double acc_bias = x[5][0];
        double angle_bias = x[6][0];

        // Derivative of state equation
        j[0][2] = cos(phi);
        j[1][2] = sin(phi);
        j[0][3] = -v * sin(phi);
        j[1][3] = v * cos(phi);
        j[3][4] = 1;
        j[2][5] = -1;
    }

    @Override
    public void processNoiseCovariance(double[][] cov) {
        cov[0][0] = 5e-4;  // Assume the position is not changing by itself - use a very small covariance
        cov[1][1] = 5e-4;  // Assume the position is not changing by itself - use a very small covariance
        cov[2][2] = 1e-4;  // Allow change in velocity can  - change by measurements
        cov[3][3] = 1e-9;  // assume phi is not changing
        cov[4][4] = 1e-2;  // Allow change in omega
        cov[5][5] = 1e1;  // Allow change in bias
        cov[6][6] = 1e-6;  // Allow change in bias

    }

}