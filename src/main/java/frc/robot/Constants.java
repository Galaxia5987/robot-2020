package frc.robot;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */


public final class Constants {
    public static final int TALON_TIMEOUT = 10;

    public static final class Intake {
        public static final boolean MASTER_INVERTED = true;
        public static final double INTAKE_SPEED = 0.5;

    }

    public static class Turret {
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
        public static final boolean STOP_TURRET = false;
        public static final double VISION_KP = 0;
        public static final double VISION_KI = 0;
        public static final double VISION_KD = 0;
        public static final double VISION_SETPOINT = 0;
      }

    static { // Runs alongside main
        if (!Robot.isRobotA) { // We want robot B constants
            replaceFields(Constants.class, BConstants.class); // Replace outer constants
            for (Class aClass : Constants.class.getDeclaredClasses()) { // Loop constants classes
                //Find the class in B Constants
                Optional<Class<?>> bClass = Arrays.stream(BConstants.class.getDeclaredClasses()).filter(c -> c.getSimpleName().equals(aClass.getSimpleName())).findAny();
                if (bClass.isEmpty()) continue; // Class isn't present
                replaceFields(aClass, bClass.get());
            }
        }
    }

    /**
     * Replaces fields between constants classes
     *
     * @param class1 Original constants class
     */
    public static void replaceFields(Class class1, Class class2) {
        //Loop and replace all fields
        for (Field f : class2.getDeclaredFields()) {
            for (Field f2 : class1.getDeclaredFields()) {
                //Loop and replace all fields
                if (f2.getName().equals(f.getName())) { // If the name is equal perform replacement

                    f2.setAccessible(true);
                    f.setAccessible(true);
                    try {
                        Field modifiersField = Field.class.getDeclaredField("modifiers");
                        modifiersField.setAccessible(true);
                        modifiersField.setInt(f2, f2.getModifiers() & ~Modifier.FINAL);
                    } catch (IllegalAccessException | NoSuchFieldException e) { // Catch relevant exceptions
                        e.printStackTrace();
                    }
                }
            }
        }

    }

    //TODO: Change values for all of the constants
    public static class Conveyor {
        public static final boolean MOTOR_INVERTED = false;
        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final int TALON_PID_SLOT = 0;
        public static final int CRUISE_VELOCITY = 0;
        public static final int CRUISE_ACCELERATION = 0;
        public static final int MAX_CURRENT = 0;
        public static final double RAMP_RATE = 0;

        public static final double FEEDER_PROXIMITY_MAX_VOLTAGE = 2; //The minimum voltage for which the sensor would see a ball
        public static final double FEEDER_PROXIMITY_MIN_VOLTAGE = 1.2; //The minimum voltage which the sensor would see in between two balls
        public static final double CONVEYOR_PROXIMITY_MAX_VOLTAGE = 0;
        public static final double CONVEYOR_PROXIMITY_MIN_VOLTAGE = 0;

        public static final int TICK_PER_METERS = 0;
        public static final int MAX_BALLS_AMOUNT = 5;

        public static final double CONVEYOR_MOTOR_FEED_VELOCITY = 0;
        public static final double CONVEYOR_MOTOR_RETURN_VELOCITY = 0;

        public static final double FEED_TIMEOUT = 5;
        public static final boolean CLOSE_GATE = false;
        public static final boolean OPEN_GATE = true;
    }
    public static class Shooter {
        public static final double KP = 0.085; // 0.13
        public static final double KI = 0.0;
        public static final double KD = 0.00;
        public static final double KF = 0.003;
        public static final int MAX_CURRENT = 35; //[A]
        public static final int TICKS_PER_ROTATION = 36;
        public static final double SHOOTING_TIME = 3.5; // [s]
        public static final double MAX_ACCELERATION = 2;
        public static final double SPEED_THRESHOLD = 0;
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
