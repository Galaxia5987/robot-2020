package frc.robot;
import edu.wpi.first.wpilibj.util.Color;
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
        public static final double[] VELOCITY_PID_SET = {0.0001, 0, 2, 0};//PID set for the velocity drive of the wheels
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
        public static final double TRACK_WIDTH = 0.72;
        public static final boolean GYRO_INVERTED = true;
        public static final double WHEEL_DIAMETER = 0.1016;
    }
    public static final class LED {
        public static final Color DEFAULT_COLOR = Color.kDeepSkyBlue;

        // Dimness settings
        public static final double MINIMAL_DIMNESS = 0;
        public static final double DIMNESS_INCREASE = 0;

        // Blinking settings
        public static final int TOTAL_BLINKS = 5;
        public static final double BLINK_PAUSE = 0.125; // TODO: Tune blink values.

    }


    public static class Autonomous {
        // Drivetrain characterization constants
        public static final double leftkS = 0.367;
        public static final double leftkV = 1.6;
        public static final double leftkA = 0.0527;

        public static final double rightkS = 0.361;
        public static final double rightkV = 1.59;
        public static final double rightkA = 0.0667;

        // Follower constants
        public static final double kBeta = 2;
        public static final double kZeta = 0.7;
    }
    public static class ColorWheel{
        public static final double[] POLY_YELLOW_RGB = {0.297, 0.541, 0.161};
        public static final double[] POLY_GREEN_RGB = {0.195, 0.526, 0.281};
        public static final double[] POLY_RED_RGB = {0.398, 0.398, 0.202};
        public static final double[] POLY_BLUE_RGB = {0.166, 0.435, 0.398};

        public static final double[] YELLOW_RGB = {0.317, 0.552, 0.127};
        public static final double[] GREEN_RGB = {0.16, 0.571, 0.269};
        public static final double[] RED_RGB = {0.492, 0.348, 0.145};
        public static final double[] BLUE_RGB = {0.132, 0.427, 0.442};

        public static final int TILES_BEFORE_SENSOR = 2; //The amount of color tiles between the robot sensor and the field sensor (for example at TBS = 2, the position would aim for RED when the FMS asks for BLUE)

        public static final double POSITION_CONTROL_TIMER = 1;
        public static final double POSITION_CONTROL_POWER = 0.2;
        public static final double ROTATION_CONTROL_POWER = 0.2;
        public static double kP = 0.3;
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
