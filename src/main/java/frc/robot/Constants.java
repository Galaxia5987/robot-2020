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
public class Constants {
    //All general constants go here
    //public static final double TIME_STEP = CONST(0.02);

    public static final int TALON_TIMEOUT_MS = 10;
    public static final double ROBOT_WIDTH = 0;

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
     * @param class2 Constants to replace with
     */
    public static void replaceFields(Class class1, Class class2) {
        //Loop and replace all fields
        for (Field f : class2.getDeclaredFields()) {
            for (Field f2 : class1.getDeclaredFields()) {
                if (f2.getName().equals(f.getName())) { // If the name is equal perform replacement
                    f2.setAccessible(true);
                    f.setAccessible(true);
                    try {
                        //Override final modifier
                        Field modifiersField = Field.class.getDeclaredField("modifiers");
                        modifiersField.setAccessible(true);
                        modifiersField.setInt(f2, f2.getModifiers() & ~Modifier.FINAL);
                        f2.set(null, f.get(null)); // Set value
                    } catch (IllegalAccessException | NoSuchFieldException e) { // Catch relevant exceptions
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    public static class ExampleSubsystem1 {
        //All of the Subsystem specific constants go here,and need to be static.

        //public static final double TICKS_PER_METER = CONST(256 / (4*0.0254*Math.PI));
        //public static final double MAX_VELOCITY = CONST(5);
    }

    public static class Climb {
        public static final int MOTION_MAGIC_VELOCITY = 0;
        public static final int MOTION_MAGIC_ACCELERATION = 0;
        public static final int TICKS_PER_METER = 1;
        public static final double[] CLIMB_PIDFA = {0, 0, 0, 0,0};//Proportional, Integral, Derivative, Feedforward, Arbitrary Feedforward
        public static final boolean IS_LEFT_MOTOR_INVERTED = false;
        public static final boolean IS_RIGHT_MOTOR_INVERTED = false;
        public static final boolean IS_LEFT_SENSOR_INVERTED = false;
        public static final boolean IS_RIGHT_SENSOR_INVERTED = false;
        public static final double HEIGHT_TOLERANCE = 0;//The tolerance between the current height to the desired height
        public static final double ANGLE_TOLERANCE = 0;//The tolerance between the current angle to the desired angle
        public static final double HEIGHT = 0;// The maximum height of the subsystem
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