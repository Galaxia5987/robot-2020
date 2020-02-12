package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.valuetuner.WebConstant;
import org.apache.commons.lang.math.DoubleRange;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public class Constants{

    public static final int TALON_TIMEOUT = 10;

    public static class Drivetrain {
        //Remember! High gear == High speed!
        public static final double WHEEL_DIAMETER = 6*0.0254;
        public static final double TRACK_WIDTH = 0.6649; // TODO: this is horizontal distance between the wheels, we might need a diagonal distance.

        public static final double LOW_TICKS_PER_METER = 4096 * (2000 / 216.) / (WHEEL_DIAMETER * Math.PI); // TICKS * RATIO / CIRCUMFERENCE
        public static final double HIGH_TICKS_PER_METER = 4096 * (2500 / 126.) / (WHEEL_DIAMETER * Math.PI); // TICKS * RATIO / CIRCUMFERENCE

        public static final double[] VELOCITY_PID_SET = {0.0, 0, 0, 0}; // PID gains set for the velocity drive of the wheels.

        //Shifter enabled constants
        public static final double SHIFTER_COOLDOWN = 0.5; // Time after shifting the shifter is not to be used.
        public static final double HIGH_ACCELERATION_THRESHOLD = 0; // Threshold for the acceleration required to go into high gear.
        public static final double LOW_ACCELERATION_THRESHOLD = 0; // Threshold for the acceleration required to go into low gear.
        public static final double HIGH_GEAR_MIN_VELOCITY = 0;
        public static final double LOW_GEAR_MIN_VELOCITY = 0;
        public static final double TURNING_TOLERANCE = 0; // Stops the robot from shifting while the robot is turning.
        public static final double GRAVITY_ACCELERATION = 9.80665;

        public static final double JOYSTICK_END_THRESHOLD = 0;

        public static final double JOYSTICK_MIN_THRESHOLD = 0.05;
    }

    public static class Autonomous {
        // Drivetrain characterization constants
        public static final double leftkS = 0.367;
        public static final double leftkV = 1.6;
        public static final double leftkA = 0.0527;
        public static final double rightkS = 0.361;
        public static final double rightkV = 1.59;
        public static final double rightkA = 0.0667;

        // Ramsete controller constants
        public static final double kBeta = 2;
        public static final double kZeta = 0.7;
    }

    public static class FieldGeometry {
        public static final Pose2d OUTER_POWER_PORT_LOCATION = new Pose2d(15.98, 5.81, new Rotation2d()); // The opponent location is (x: 0, y: 2.4).
        public static final Pose2d INNER_POWER_PORT_LOCATION = new Pose2d(15.98 + 0.78, 5.81, new Rotation2d()); // The opponent location is (x: -0.78, y: 2.4).
    }

    public static final class Intake {
        public static final WebConstant INTAKE_POWER = new WebConstant("intakePower", 0.4);
        public static final WebConstant OUTTAKE_POWER = new WebConstant("intakeOutPower", 0.2);
    }

    public static class Conveyor {
        public static final double TICK_PER_METERS = 0.0382 * 4096;

        public static final double KP = 0.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final int CRUISE_VELOCITY = 0;
        public static final int CRUISE_ACCELERATION = 0;
        public static final double RAMP_RATE = 0;

        public static final WebConstant PULSE_INTERVAL = new WebConstant("pulseInterval", 0.1);
        public static final double CONVEYOR_MOTOR_FEED_POWER = 0;
        public static final WebConstant CONVEYOR_MOTOR_OPEN_FEED_POWER = new WebConstant("conveyorOpenFeedPower", 0.7);
        public static final WebConstant FUNNEL_MOTOR_FEED_POWER = new WebConstant("funnelFeedPower", 0.3);
        public static final WebConstant CONVEYOR_MOTOR_INTAKE_POWER = new WebConstant("conveyorIntakePower", 0.7);
        public static final WebConstant CONVEYOR_OUTTAKE_POWER = new WebConstant("conveyorOuttakePower", 0.5);
        public static final WebConstant FUNNEL_OUTTAKE_POWER = new WebConstant("funnelOuttakePower", 0.5);

        public static final double CONVEYOR_MOTOR_RETURN_POWER = 0;
        public static final double FEED_TIMEOUT = 5;

        public static final double INTAKE_PROXIMITY_MAX_VOLTAGE = 2; //The minimum voltage for which the sensor would see a ball
        public static final double INTAKE_PROXIMITY_MIN_VOLTAGE = 1.2; //The minimum voltage which the sensor would see in between two balls
        public static final double SHOOTER_PROXIMITY_MAX_VOLTAGE = 0;
        public static final double SHOOTER_PROXIMITY_MIN_VOLTAGE = 0;

        public static final int MAX_BALLS_AMOUNT = 5;
        public static final int STARTING_AMOUNT = 3;

    }

    public static class Turret {
        public static final double VISION_TIMEOUT_SECONDS = 1;

        public static final double TICKS_PER_DEGREE = 4096/360.0;

        public static final DoubleRange ALLOWED_ANGLES = new DoubleRange(0, 200);

        public static final double STARTING_ANGLE = 90;
        public static final int STARTING_POSITION = 2680;

        public static final int POSITION_PID_SLOT = 0;
        public static final int MOTION_MAGIC_PID_SLOT = 1;
        
        public static double KP = 3.5;
        public static double KI = 0.01;
        public static double KD = 50;
        public static double KF = 0;
        public static double ALLOWABLE_ERROR = 0.3;

        public static double MOTION_MAGIC_KP = 3.5;
        public static double MOTION_MAGIC_KI = 0.0005;
        public static double MOTION_MAGIC_KD = 2.3;
        public static double MOTION_MAGIC_KF = 0.00912;

        public static final int MOTION_MAGIC_CRUISE_VELOCITY = 20;
        public static final int MOTION_MAGIC_ACCELERATION = 16;

        public static final WebConstant CW_FRICTION_COEFFICIENT = new WebConstant("visionFrictionCW", 0.793 / 12);
        public static final WebConstant CCW_FRICTION_COEFFICIENT = new WebConstant("visionFrictionCCW", 0.978 / 12);
        public static final WebConstant DIRECT_VISION_KP = new WebConstant("visionKp", 0.01);
        public static final WebConstant DIRECT_VISION_KI = new WebConstant("visionKi", 0.001);
        public static final WebConstant DIRECT_VISION_KD = new WebConstant("visionKd", 0);

        public static final double TURRET_JOYSTICK_SPEED = 10; //Coefficient of the joystick value per degree.

        public static final int MAX_CURRENT = 35; // [A]
        public static final double ANGLE_THRESHOLD = 1;

        public static final int BACKLASH_ANGLE = 0; // The angle in which the motor moves without the mechanical system moving when switching direction
        public static final int VELOCITY_MINIMUM = 0; // Minimum velocity to indicate actual movement of the system instead of just small error

        public static final double CONTROL_MODE_THRESHOLD = 50;
    }


    public static class Shooter {
        public static final double TICKS_PER_ROTATION = 4096;
        public static final double KP = 0.3; // 0.13
        public static final double KI = 0.0;

        public static final double KD = 1.5;
        public static final double KF = 0.014;

        public static final int MAX_CURRENT = 35; //[A]
        public static final double SHOOTING_TIME = 3.5; // [s]
        public static final double VELOCITY_TOLERANCE = 0; // the acceptable velocity threshold error of the shooter
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

        public static final int TILES_BEFORE_SENSOR = 2; // The amount of color tiles between the robot sensor and the field sensor (for example at TBS = 2, the position would aim for RED when the FMS asks for BLUE).

        public static final double POSITION_CONTROL_TIMER = 1;
        public static final double POSITION_CONTROL_POWER = 0.2;
        public static final double ROTATION_CONTROL_POWER = 0.2;
        public static double kP = 0.3;
    }

    public static class Climber {
        public static final double TICKS_PER_METER = 4096 * 0.03 * Math.PI * 100; // TICKS * diameter * pi

        public static final double[] CLIMB_PIDF = {0.12, 0, 0, 0}; // Proportional, Integral, Derivative, Feedforward
        public static final double[] CLIMB_RELEASE_PIDF = {0.12, 0, 0, 0}; // Proportional, Integral, Derivative, Feedforward

        public static final double[] DELTA_PID = {0, 0, 0}; // Proportional, Integral, Derivative

        public static final double MAX_HEIGHT = 0.75; // The allowed maximum height of the subsystem.

        public static final double ARBITRARY_FEEDFORWARD = 0;

        public static final double RAMP_RATE = 0;

        public static final WebConstant CLIMB_HEIGHT = new WebConstant("climbSetpointHeight", 0.3);

        public static final double ALLOWED_HEIGHT_TOLERANCE = 0; // The allowed tolerance between the current height to the desired height.
        public static final double ALLOWED_ANGLE_TOLERANCE = 0; // The allowed tolerance between the current angle to the desired angle.
        public static final WebConstant MODIFY_JOYSTICK_RATE = new WebConstant("climbJoystickRate", 0.7); // The factor which the value of the joystick is multiplied by to calculate the change rate.
        public static final double MAX_DIFFERENCE = 2; // The maximal difference between the two sides of the climber.
        public static final double DISTANCE_BETWEEN_RODS = 0; // The distance between both climbing rods.
    }

    static { // Runs alongside main
        if (!Robot.isRobotA) { // We want robot B constants
            replaceFields(Constants.class, BConstants.class); // Replace outer constants
            for (Class aClass : Constants.class.getDeclaredClasses()) { // Loop constants classes
                // Find the class in B Constants
                Optional<Class<?>> bClass = Arrays.stream(BConstants.class.getDeclaredClasses()).filter(c -> c.getSimpleName().equals(aClass.getSimpleName())).findAny();
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


//Anything in this class will replace the original constants when boolean is true
class BConstants {
    //General constants to be replaced

    public static final class Intake {

    }

    public static class Turret {

    }

    public static class Conveyor {

    }

    public static class Shooter {

    }

}
