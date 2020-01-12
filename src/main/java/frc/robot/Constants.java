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
    public static final class Intake {
        public static final boolean INTAKE_MOTOR_INVERTED = true;

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
