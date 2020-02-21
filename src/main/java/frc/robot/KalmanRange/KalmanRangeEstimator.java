package frc.robot.KalmanRange;

import frc.robot.subsystems.drivetrain.EKF.KalmanFilter;
import frc.robot.subsystems.drivetrain.EKF.ObservationModel;
import frc.robot.subsystems.drivetrain.KalmanLocalization.OdometryInertialObservation;
import frc.robot.subsystems.drivetrain.KalmanLocalization.OdometryInertialProcess;

public class KalmanRangeEstimator {
    public  KalmanFilter filter;
    private Linear1dProcessModel process;
    private Linear1dObservationModel observation;
    double  m_proportional_range_err = 0.05; // Initiliaze to 5% of range

    /**
     * Constructs a KalmanRangeEstimator object.
     *
     * @param proportional_range_err         expected vision error relative to range. For example, 0.05 is 5% error.
     */
    public KalmanRangeEstimator(double proportional_range_err)
    {
        observation = new Linear1dObservationModel();
        process = new Linear1dProcessModel();
        filter = new KalmanFilter(process);

    }


    /**
     * Returns the current best estimate of the range
     */
    public double GetRange() {return filter.model.state_estimate.data[0][0];}


    /**
     * Resets the filter to initiate state - when range measurements stopped.
     */
    public void Reset() {
        filter.model.initialState(filter.model.state_estimate.data);
        filter.model.initialStateCovariance(filter.model.estimate_covariance.data);
    }

    /**
     * Filters the range
     *
     * @param R - measured range in meters
     * @param time - current time in seconds
     * @param R_valid - if the measurement is valid. If invalid the filter will continue on prediction
     */
    public double FilterRange(double R,double time, boolean R_valid)
    {
        if (R_valid) {
            observation.SetObservationErrorStd(R * m_proportional_range_err);
        }
        else
        {
            observation.SetObservationErrorStd(1000);
        }
        observation.setPosition(R);

        filter.update(time,observation);

        return filter.model.state_estimate.data[0][0];
    }
}


