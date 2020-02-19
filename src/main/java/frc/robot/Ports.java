package frc.robot;


import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;

import static org.apache.commons.lang3.ObjectUtils.CONST;

public class Ports {
    public static final int TALON_PID_SLOT = 0;

    public static class Conveyor {
        public static final int FUNNEL = 19;
        public static final int MOTOR = 21;
        public static final boolean MOTOR_INVERTED = CONST(false);
        public static final boolean FUNNEL_INVERTED = CONST(true);
        public static final int SHOOTER_PROXIMITY = 1;
        public static final int FORWARD_GATE = 5;
        public static final int REVERSE_GATE = 4;
        public static final int GATE = 1; // Port of the stopper solenoid.
        public static final boolean IS_GATE_REVERSED = CONST(false);
    }

    public static final class Intake {
        public static final int MOTOR = 20;
        public static final boolean MOTOR_INVERTED = CONST(true);
        public static final boolean IS_FORWARD_OPEN = CONST(true); //if kForward of the solenoid opens the intake or reverse.
        public static final int FOLD_SOLENOID_REVERSE = 7;
        public static final int FOLD_SOLENOID_FORWARD = 6;
        public static final int SOLENOID = 0;
        public static final boolean IS_SOLENOID_REVERSED = CONST(false);
    }

    public static class Shooter {
        public static final int MASTER = 23;
        public static final int SLAVE_1 = 24;
        public static final int SLAVE_2 = 25;
        public static final boolean IS_MASTER_INVERTED = CONST(true);
        public static final boolean IS_SLAVE_1_INVERTED = CONST(false);
        public static final boolean IS_SLAVE_2_INVERTED = CONST(true);
        public static final boolean IS_ENCODER_INVERTED = CONST(true);
    }

    public static class Turret {
        public static final int MOTOR = 22;
        public static final boolean IS_MOTOR_INVERTED = CONST(true);
        public static final boolean IS_ENCODER_INVERTED = CONST(true);

        public static final boolean ENABLE_SOFT_LIMITS = CONST(true);
        public static final boolean DISABLE_SOFT_LIMITS_ON_DISCONNECT = CONST(true);
    }

    public static class Drivetrain {
        public static final int LEFT_MASTER = 10;
        public static final int LEFT_SLAVE = 11;
        public static final int RIGHT_MASTER = 12;
        public static final int RIGHT_SLAVE = 13;

        public static final boolean LEFT_MASTER_INVERTED = CONST(false);
        public static final boolean LEFT_SLAVE_INVERTED = CONST(false);

        public static final boolean RIGHT_MASTER_INVERTED = CONST(true);
        public static final boolean RIGHT_SLAVE_INVERTED = CONST(true);

        public static final int SHIFTER_FORWARD_PORT = 1;
        public static final int SHIFTER_REVERSE_PORT = 0;
        public static final int SHIFTER_PORT = 2;
        public static final boolean IS_SHIFTER_REVERSED = CONST(false);
    }

    public static class ColorWheel {
        public static final int MOTOR = 30;
    }

    public static class Climber {
        public static final int LEFT_MOTOR = 40;
        public static final int RIGHT_MOTOR = 41;
        public static final int STOPPER_FORWARD = 3;
        public static final int STOPPER_REVERSE = 2;
        public static final int STOPPER = 3;
        public static final boolean LEFT_MOTOR_INVERTED = CONST(false);
        public static final boolean RIGHT_MOTOR_INVERTED = CONST(true);
        public static final boolean LEFT_ENCODER_INVERTED = CONST(true);
        public static final boolean RIGHT_ENCODER_INVERTED = CONST(true);
        public static final boolean IS_STOPPER_REVERSED = CONST(false);
    }

    static { // Runs alongside main
        if (!Robot.isRobotA) { // We want robot B constants
            replaceFields(Ports.class, BPorts.class); // Replace outer constants
            for (Class aClass : Constants.class.getDeclaredClasses()) { // Loop constants classes
                // Find the class in B Constants
                Optional<Class<?>> bClass = Arrays.stream(BPorts.class.getDeclaredClasses()).filter(c -> c.getSimpleName().equals(aClass.getSimpleName())).findAny();
                if (bClass.isEmpty()) continue; // Class isn't present
                replaceFields(aClass, bClass.get());
            }
        }
    }

    /**
     * Replaces fields between constants classes.
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
                        f2.set(null, f.get(null));
                    } catch (IllegalAccessException | NoSuchFieldException e) { // Catch relevant exceptions
                        e.printStackTrace();
                    }
                }
            }
        }
    }
}

class BPorts{
    public static class Shooter {
        public static final boolean IS_MASTER_INVERTED = true;
        public static final boolean IS_SLAVE_1_INVERTED = false;
        public static final boolean IS_SLAVE_2_INVERTED = true;
        public static final boolean IS_ENCODER_INVERTED = true;
    }


    public static class Turret {
        public static final boolean IS_MOTOR_INVERTED = true;
        public static final boolean IS_ENCODER_INVERTED = true;

        public static final boolean ENABLE_SOFT_LIMITS = true;
        public static final boolean DISABLE_SOFT_LIMITS_ON_DISCONNECT = true;
    }

    public static class Drivetrain {
        public static final boolean LEFT_MASTER_INVERTED = false;
        public static final boolean LEFT_SLAVE_INVERTED = false;

        public static final boolean RIGHT_MASTER_INVERTED = true;
        public static final boolean RIGHT_SLAVE_INVERTED = true;

        public static final boolean IS_SHIFTER_REVERSED = false;
    }

    public static class ColorWheel {
    }

    public static class Climber {
        public static final boolean LEFT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_MOTOR_INVERTED = true;
        public static final boolean LEFT_ENCODER_INVERTED = true;
        public static final boolean RIGHT_ENCODER_INVERTED = true;
        public static final boolean IS_STOPPER_REVERSED = false;
    }
}

