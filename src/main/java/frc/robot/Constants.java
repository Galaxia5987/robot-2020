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

    public static class Turret {
        public static final int TALON_TIMEOUT = 10;
        public static final int TALON_PID_SLOT = 0;
        public static final int MAX_CURRENT = 35;
        public static final double TICKS_PER_DEGREE = 1;
        public static double KP = 0;
        public static double KI = 0;
        public static double KD = 0;
        public static double KF = 0;
        public static final int CRUISE_VELOCITY= 0;
        public static final int CRUISE_ACCELERATION = 0;
        public static final double HALL_EFFECT_POSITION_1 = 0; // in degrees
        public static final double HALL_EFFECT_POSITION_2 = 0; // in degrees
        public static final double ANGLE_THRESHOLD = 5;
        public static final double MAXIMUM_POSITION = -360;
        public static final double MINIMUM_POSITION = 360;

    }

}
