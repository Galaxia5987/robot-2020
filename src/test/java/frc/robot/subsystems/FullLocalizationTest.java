package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.drivetrain.FullLocalization;
import org.junit.Before;
import org.junit.Test;

import static java.lang.Math.*;
import static org.junit.Assert.*;

public class FullLocalizationTest {

    FullLocalization localization;
    DifferentialDriveOdometry wpi_localization;
    double m_width = 1;
    @Before
    public void setUp() throws Exception {
        Rotation2d angle = new Rotation2d(0);
        localization = new FullLocalization(angle,m_width);
        wpi_localization = new DifferentialDriveOdometry(angle);
    }

    @Test
    public void updateNominal() {
        Rotation2d angle = new Rotation2d(0);

        double left ;
        double right;


        double dt = 2e-2; // 20 msec timestep
        double time = dt;
        double[] acc =   {0.1,0.2,0.3,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1,0.8,0.8,0.8,0.7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0   , 0  ,0   ,0  }; // m/s^2
        double[] omega = {0  ,0  ,0  ,0  ,  0,0  ,0,0,0,0,0,0,0,0,0,0  ,0  , 0 , 0 ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,0.02,0.02,0.02}; // rad/s

        double[] v = new double[acc.length];
        double[] phi = new double[acc.length];
        double[] x = new double[acc.length];
        double[] y = new double[acc.length];
        double[] left_track = new double[acc.length];
        double[] right_track = new double[acc.length];
        double[] right_track_pos_x = new double[acc.length];
        double[] right_track_pos_y = new double[acc.length];
        double[] left_track_pos_x = new double[acc.length];
        double[] left_track_pos_y = new double[acc.length];

        v[0] = 0;
        phi[0] = 0;
        x[0] = 0;
        y[0] = 0;
        right_track_pos_x[0] = x[0] + m_width/2*sin(-phi[0]);
        right_track_pos_y[0] = y[0] - m_width/2*cos(-phi[0]);
        left_track_pos_x[0] = x[0] - m_width/2*sin(-phi[0]);
        left_track_pos_y[0] = y[0] + m_width/2*cos(-phi[0]);
        right_track[0]=0;
        left_track[0]=0;


        for (int i=1; i < acc.length; i ++ )
        {
            omega[i] = omega[i]+0.1; // Create constant angular rate
            v[i]=v[i-1]+dt*acc[i-1];
            phi[i]=phi[i-1]+dt*omega[i-1];
            x[i] = x[i-1] + dt*(v[i] + v[i-1])/2*cos((phi[i]+phi[i-1])/2);
            y[i] = y[i-1] + dt*(v[i] + v[i-1])/2*sin((phi[i]+phi[i-1])/2);


            right_track_pos_x[i] = x[i] + m_width/2*sin(-phi[i]);
            right_track_pos_y[i] = y[i] - m_width/2*cos(-phi[i]);
            left_track_pos_x[i] = x[i] - m_width/2*sin(-phi[i]);
            left_track_pos_y[i] = y[i] + m_width/2*cos(-phi[i]);

            left_track[i] =left_track[i-1] +  sqrt( pow(left_track_pos_x[i]-left_track_pos_x[i-1],2) + pow(left_track_pos_y[i]-left_track_pos_y[i-1],2)   );
            right_track[i] = right_track[i-1]  + sqrt( pow(right_track_pos_x[i]-right_track_pos_x[i-1],2) + pow(right_track_pos_y[i]-right_track_pos_y[i-1],2)   );
        }



        Pose2d ref_pose;
        Pose2d ekf_pose;

        for (int i =1; i<acc.length ; i ++) {
            angle = Rotation2d.fromDegrees( toDegrees( phi[i]));
            left = left_track[i];
            right = right_track[i];

            ref_pose = wpi_localization.update(angle, left, right);
            ekf_pose = localization.update(angle, left, right, acc[i] + 2*(random()-0.5),time);


            if ( i> 8) { // ignore first cycles, before KF is stabilized
                assertEquals(ref_pose.getTranslation().getX(), ekf_pose.getTranslation().getX(), 1e-2);
                assertEquals(ref_pose.getTranslation().getY(), ekf_pose.getTranslation().getY(), 1e-2);
                assertEquals(ref_pose.getRotation().getDegrees(), ekf_pose.getRotation().getDegrees(), 1);
                assertEquals(v[i], localization.filter.model.state_estimate.data[2][0], 0.07);
                assertEquals(omega[i], localization.filter.model.state_estimate.data[4][0], 0.07);
                assertEquals(phi[i],ekf_pose.getRotation().getRadians(), 0.05);
            }
            time += dt;
        }
        }

    @Test
    public void updateAccBias() {
        Rotation2d angle = new Rotation2d(0);

        double left ;
        double right;


        double dt = 2e-2; // 20 msec timestep
        double time = dt;
        double[] acc =   {0.1,0.2,0.3,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1,0.8,0.8,0.8,0.7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0   , 0  ,0   ,0  }; // m/s^2
        double[] omega = {0  ,0  ,0  ,0  ,  0,0  ,0,0,0,0,0,0,0,0,0,0  ,0  , 0 , 0 ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,0.02,0.02,0.02}; // rad/s

        double[] v = new double[acc.length];
        double[] phi = new double[acc.length];
        double[] x = new double[acc.length];
        double[] y = new double[acc.length];
        double[] left_track = new double[acc.length];
        double[] right_track = new double[acc.length];
        double[] right_track_pos_x = new double[acc.length];
        double[] right_track_pos_y = new double[acc.length];
        double[] left_track_pos_x = new double[acc.length];
        double[] left_track_pos_y = new double[acc.length];

        v[0] = 0;
        phi[0] = 0;
        x[0] = 0;
        y[0] = 0;
        right_track_pos_x[0] = x[0] + m_width/2*sin(-phi[0]);
        right_track_pos_y[0] = y[0] - m_width/2*cos(-phi[0]);
        left_track_pos_x[0] = x[0] - m_width/2*sin(-phi[0]);
        left_track_pos_y[0] = y[0] + m_width/2*cos(-phi[0]);
        right_track[0]=0;
        left_track[0]=0;


        for (int i=1; i < acc.length; i ++ )
        {
            omega[i] = omega[i]+0.1; // Create constant angular rate
            v[i]=v[i-1]+dt*acc[i-1];
            phi[i]=phi[i-1]+dt*omega[i-1];
            x[i] = x[i-1] + dt*(v[i] + v[i-1])/2*cos((phi[i]+phi[i-1])/2);
            y[i] = y[i-1] + dt*(v[i] + v[i-1])/2*sin((phi[i]+phi[i-1])/2);


            right_track_pos_x[i] = x[i] + m_width/2*sin(-phi[i]);
            right_track_pos_y[i] = y[i] - m_width/2*cos(-phi[i]);
            left_track_pos_x[i] = x[i] - m_width/2*sin(-phi[i]);
            left_track_pos_y[i] = y[i] + m_width/2*cos(-phi[i]);

            left_track[i] =left_track[i-1] +  sqrt( pow(left_track_pos_x[i]-left_track_pos_x[i-1],2) + pow(left_track_pos_y[i]-left_track_pos_y[i-1],2)   );
            right_track[i] = right_track[i-1]  + sqrt( pow(right_track_pos_x[i]-right_track_pos_x[i-1],2) + pow(right_track_pos_y[i]-right_track_pos_y[i-1],2)   );
        }



        Pose2d ref_pose;
        Pose2d ekf_pose;

        for (int i =1; i<acc.length ; i ++) {
            angle = Rotation2d.fromDegrees( toDegrees( phi[i]));
            left = left_track[i];
            right = right_track[i];

            ref_pose = wpi_localization.update(angle, left, right);
            ekf_pose = localization.update(angle, left, right, acc[i] + 0.5*(random()-0.5) + 0.2,time);


            if ( i> 8) { // ignore first cycles, before KF is stabilized
                assertEquals(ref_pose.getTranslation().getX(), ekf_pose.getTranslation().getX(), 1e-2);
                assertEquals(ref_pose.getTranslation().getY(), ekf_pose.getTranslation().getY(), 1e-2);
                assertEquals(ref_pose.getRotation().getDegrees(), ekf_pose.getRotation().getDegrees(), 1);
                assertEquals(v[i], localization.filter.model.state_estimate.data[2][0], 0.07);
                assertEquals(omega[i], localization.filter.model.state_estimate.data[4][0], 0.07);
                assertEquals(phi[i],ekf_pose.getRotation().getRadians(), 0.05);
            }
            time += dt;
        }

        // Verify acc bias esitmate is correct
        assertEquals(0.2,localization.filter.model.state_estimate.data[5][0], 0.05);

    }


    @Test
    public void encoderValid()
    {
        Rotation2d angle = new Rotation2d(20);
        Pose2d pose = new Pose2d(0,0,angle);

        localization.resetPosition(pose,angle,0);
        assertEquals(true,localization.EncoderValid(angle,0,0));
        assertEquals(false,localization.EncoderValid(angle,1,0));
        assertEquals(false,localization.EncoderValid(angle,0.1,0));
    }

    @Test
    public void updateNoEncoder() {
        Rotation2d angle = new Rotation2d(0);

        double left ;
        double right;


        double dt = 2e-2;
        double time = dt;
        double[] acc =   {0.1,0.2,0.3,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1,0.8,0.8,0.8,0.7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0   , 0  ,0   ,0  }; // m/s^2
        double[] omega = {0  ,0  ,0  ,0  ,  0,0  ,0,0,0,0,0,0,0,0,0,0  ,0  , 0 , 0 ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.01,0.02,0.02,0.02}; // rad/s

        double[] v = new double[acc.length];
        double[] phi = new double[acc.length];
        double[] x = new double[acc.length];
        double[] y = new double[acc.length];
        double[] left_track = new double[acc.length];
        double[] right_track = new double[acc.length];
        double[] right_track_pos_x = new double[acc.length];
        double[] right_track_pos_y = new double[acc.length];
        double[] left_track_pos_x = new double[acc.length];
        double[] left_track_pos_y = new double[acc.length];

        v[0] = 0;
        phi[0] = 0;
        x[0] = 0;
        y[0] = 0;
        right_track_pos_x[0] = x[0] + m_width/2*sin(-phi[0]);
        right_track_pos_y[0] = y[0] - m_width/2*cos(-phi[0]);
        left_track_pos_x[0] = x[0] - m_width/2*sin(-phi[0]);
        left_track_pos_y[0] = y[0] + m_width/2*cos(-phi[0]);
        right_track[0]=0;
        left_track[0]=0;



        for (int i=1; i < acc.length; i ++ )
        {
            omega[i] = omega[i]+0.1; // Create constant angular rate
            v[i]=v[i-1]+dt*acc[i-1];
            phi[i]=phi[i-1]+dt*omega[i-1];
            x[i] = x[i-1] + dt*(v[i] + v[i-1])/2*cos((phi[i]+phi[i-1])/2);
            y[i] = y[i-1] + dt*(v[i] + v[i-1])/2*sin((phi[i]+phi[i-1])/2);


            right_track_pos_x[i] = x[i] + m_width/2*sin(-phi[i]);
            right_track_pos_y[i] = y[i] - m_width/2*cos(-phi[i]);
            left_track_pos_x[i] = x[i] - m_width/2*sin(-phi[i]);
            left_track_pos_y[i] = y[i] + m_width/2*cos(-phi[i]);

            left_track[i] =left_track[i-1] +  sqrt( pow(left_track_pos_x[i]-left_track_pos_x[i-1],2) + pow(left_track_pos_y[i]-left_track_pos_y[i-1],2)   );
            right_track[i] = right_track[i-1]  + sqrt( pow(right_track_pos_x[i]-right_track_pos_x[i-1],2) + pow(right_track_pos_y[i]-right_track_pos_y[i-1],2)   );
        }



        Pose2d ref_pose;
        Pose2d ekf_pose;

        for (int i =1; i<acc.length ; i ++) {

            if (i>= 15) // Insert wrong encoder value
            {
                left_track[i] += 1;
            }

            angle = Rotation2d.fromDegrees( toDegrees( phi[i]));
            left = left_track[i];
            right = right_track[i];

            ref_pose = wpi_localization.update(angle, left, right);
            ekf_pose = localization.update(angle, left, right, acc[i] + 2*(random()-0.5),time);


            if ( i> 8) { // ignore first cycles, before KF is stabilized
                assertEquals(x[i], ekf_pose.getTranslation().getX(), 1e-2);
                assertEquals(y[i], ekf_pose.getTranslation().getY(), 1e-2);
                assertEquals(v[i], localization.filter.model.state_estimate.data[2][0], 0.07);
                assertEquals(omega[i], localization.filter.model.state_estimate.data[4][0], 0.07);
                assertEquals(phi[i],ekf_pose.getRotation().getRadians(), 0.05);
            }

            if (i>15)
            {
                // WPI localization should be wrong
                assertNotEquals(ref_pose.getTranslation().getX(), ekf_pose.getTranslation().getX(), 1e-2);

            }
            time += dt;
        }
    }

}