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


    public static class Shooter {
        public static final int TALON_PID_SLOT = 0;
        public static final double KP = 0.085; // 0.13
        public static final double KI = 0.0;
        public static final double KD = 0.00;
        public static final double KF = 0.003;
        public static final int TALON_TIMEOUT = 10;
        public static final int MAX_CURRENT = 35; //[A]
        public static final double RADIUS = 0.05; // [m]
        public static final int TICKS_PER_ROTATION = 36;
        public static final int TICKS_PER_SECOND = 1;
        public static final double g = 9.80665; // [m/sec^2]
        public static final double TARGET_DISTANCE = 2; // [m]
        public static final double SHOOTER_HEIGHT = 0.4; // [m]
        public static final double TARGET_HEIGHT = 2.5; // [m]
        public static final double ANGLE = 80; // [deg]
        public static final double VELOCITY_DIFFERENCE = 830;
        public static final double SHOOTING_TIME = 3.5;
        public static final double TARGET_RPM = 500;
        public static final double PERCENT_THRESHOLD = 10;
        public static final double RAMP_RATE = 2;
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