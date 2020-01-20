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

    public static class Drivetrain {
        public static final double[] VELOCITY_PID_SET = {0, 0, 0, 0};//PID set for the velocity drive of the wheels
        public static final double SHIFTER_COOLDOWN = 0.5;//Time after shifting the shifter is not to be used
        public static final double HIGH_ACCELERATION_THRESHOLD = 0;//Threshold for the acceleration required to go into high gear
        public static final double LOW_ACCELERATION_THRESHOLD = 0;//Threshold for the acceleration required to go into low gear
        public static final double TURNING_TOLERANCE = 0;//Stops the robot from shifting while the robot is turning
        public static final int LOW_TICKS_PER_METER = 28914;//unit conversion while the robot is on low gear
        public static final int HIGH_TICKS_PER_METER = 0;//unit conversion while the robot is on high gear
        public static final double HIGH_GEAR_MIN_VELOCITY = 0;
        public static final double LOW_GEAR_MIN_OUTPUT = 0;
        public static final double GRAVITY_ACCELERATION = 9.80665;
        public static final boolean RIGHT_MASTER_INVERTED = true;
        public static final boolean RIGHT_SLAVE_INVERTED = true;
        public static final double TRACK_WIDTH = 0;
    }

    public static class Autonomous {
        // Drivetrain characterization constants
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        // Follower constants
        public static final double kBeta = 0;
        public static final double kZeta = 0;
    }

    public static class ExampleSubsystem1 {
        //All of the Subsystem specific constants go here,and need to be static.

        //public static final double TICKS_PER_METER = CONST(256 / (4*0.0254*Math.PI));
        //public static final double MAX_VELOCITY = CONST(5);
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
