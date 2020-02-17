package frc.robot.subsystems;

import frc.robot.KalmanRange.KalmanRangeEstimator;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class KalmanRangeEstimatorTest {


    @Test
    public void getRange() {
        KalmanRangeEstimator filter = new KalmanRangeEstimator(0.05);

        double R = 10;
        double dt = 1 / 50.0;

        for (int i = 0; i < 20; i++) {
            filter.FilterRange(R, i * dt, true);
        }

        assertEquals(R, filter.GetRange(), 0.001);

    }

    @Test
    public void reset() {
        Random r = new Random();

        KalmanRangeEstimator filter = new KalmanRangeEstimator(0.05);

        double R = 10;
        double dt = 1 / 50.0;

        for (int i = 0; i < 20; i++) {
            filter.FilterRange(R + r.nextGaussian()*0.05, i * dt, true);
        }

        filter.Reset();

        filter.FilterRange(5, 20 * dt, true);

        assertEquals(5, filter.GetRange(), 0.01);

    }

    @Test
    public void filterRange() {

        Random r = new Random();

        KalmanRangeEstimator filter = new KalmanRangeEstimator(0.05);

        double R = 10;
        double dt = 1 / 50.0;

        for (int i = 0; i < 20; i++) {
            filter.FilterRange(R + r.nextGaussian()*0.05, i * dt, true);
        }

        assertEquals(R, filter.GetRange(), 0.1);



    }

    @Test
    public void filterRangeSpeed() {

        Random r = new Random();

        KalmanRangeEstimator filter = new KalmanRangeEstimator(0.05);

        double R = 10;
        double V = 5;
        double dt = 1 / 50.0;

        for (int i = 0; i < 40; i++) {
            double time = i * dt;
            filter.FilterRange(R + r.nextGaussian()*0.05 + V*time, time , true);
        }

        assertEquals(V, filter.filter.model.state_estimate.data[1][0] , 0.1);

    }

    @Test
    public void filterRangeSpeedNotValidRange() {

        Random r = new Random();

        KalmanRangeEstimator filter = new KalmanRangeEstimator(0.05);

        double R = 10;
        double V = 5;
        double dt = 1 / 50.0;

        for (int i = 0; i < 40; i++) {
            double time = i * dt;
            boolean valid = true;
            if ( i > 30 ) // invalidate the measurements
            {
                valid = false;
            }

            filter.FilterRange(R + r.nextGaussian()*0.05 + V*time, time , valid);
        }

        // The position must be correct despite no range data for 10 cycles
        assertEquals(R+V*40*dt, filter.GetRange() , 0.2);

        assertEquals(V, filter.filter.model.state_estimate.data[1][0] , 0.15);

    }
}