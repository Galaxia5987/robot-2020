package frc.robot;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public final class Constants {

    public static class Turret {
        public static final int TALON_TIMEOUT = 10;
        public static final int TALON_PID_SLOT = 0;
        public static final int MAX_CURRENT = 35; // [A]
        public static final int TICKS_PER_DEGREE = 1;
        public static final int MOTION_MAGIC_CRUISE_VELOCITY = 0;
        public static final int MOTION_MAGIC_ACCELERATION = 0;
        public static final double HALL_EFFECT_POSITION_1 = 0; // in degrees, the two different positions are if the turret has turned a full circle or not
        public static final double HALL_EFFECT_POSITION_2 = 0; // in degrees
        public static final double ANGLE_THRESHOLD = 1;
        public static final double MAXIMUM_POSITION = 360;
        public static final double MINIMUM_POSITION = -360;
        public static double KP = 0;
        public static double KI = 0;
        public static double KD = 0;
        public static double KF = 0;

    }

}

//Anything in this class will replace the original constants when boolean is true
class BConstants {
    //General constants to be replaced
    //public static final double TIME_STEP = CONST(0.4);

    public static class ExampleSubsystem1 {
        //public static final double TICKS_PER_METER = CONST(512 / (4*0.0254*Math.PI));
        //public static final double MAX_VELOCITY = CONST(10);
    }

}