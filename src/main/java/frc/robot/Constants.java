/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Shooter {
        public static final int TALON_PID_SLOT = 0;
        public static final double KP = 0.085; // 0.13
        public static final double KI = 0.0;
        public static final double KD = 0.00;
        public static final double KF = 0.003;
        public static final int TALON_TIMEOUT = 10;
        public static final int MAX_CURRENT = 35; //[A]
        public static final double RADIUS = 0.05; // [m]
        public static final double TICKS_PER_ROTATION = 36;
        public static final double HEIGHT = 0; // [m]
        public static final double ANGLE = 45; // [deg]
        public static final double g = 9.80665; // [m/sec^2]
        public static final double TARGET_DISTANCE = 2; // [m]
        public static final double SHOOTING_TIME = 3.5;
        public static final double TARGET_RPM = 500;
        public static final double PERCENT_THRESHOLD = 10;
        public static final double RAMP_RATE = 2;
    }

}
