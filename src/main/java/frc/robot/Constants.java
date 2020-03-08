package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.valuetuner.WebConstant;
import org.apache.commons.lang.math.DoubleRange;

import static org.apache.commons.lang3.ObjectUtils.CONST;

/**
 * A class holding all of the constants of every mechanism on the robot.
 * Place global constants in this class, and mechanism-specific constants inside their respective mechanism subclass.
 * When accessing a mechanism-specific port, call Constants.[MECHANISM].[CONSTANT]
 */
public class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static double BACK_BUMPER_TO_CENTER = 0.4895;

    public static class Drivetrain {
        //Remember! High gear == High speed!
        public static final double WHEEL_DIAMETER = 6 * 0.0254;
        public static final double TRACK_WIDTH = 0.6649; // TODO: this is horizontal distance between the wheels, we might need a diagonal distance.

        public static final double LOW_TICKS_PER_METER = 2048. * (2500 / 126.) / (WHEEL_DIAMETER * Math.PI); // TICKS * RATIO / CIRCUMFERENCE
        public static final double HIGH_TICKS_PER_METER = 2048. * (2000 / 216.) / (WHEEL_DIAMETER * Math.PI); // TICKS * RATIO / CIRCUMFERENCE

        // PID gains set for the velocity drive of the wheels.
        public static final double KP = CONST(0);
        public static final double KI = CONST(0);
        public static final double KD = CONST(0);
        public static final double KF = CONST(0);

        //Shifter enabled constants
        public static final double SHIFTER_COOLDOWN = 0.5; // Time after shifting the shifter is not to be used.
        public static final double TURNING_TOLERANCE = 1; // Stops the robot from shifting while the robot is turning.
        public static final double SHIFT_SPEED_TOLERANCE = 0.5; // Stops the robot from shifting while the robot is too fast
        public static final double GRAVITY_ACCELERATION = 9.80665;

        public static final double JOYSTICK_END_THRESHOLD = 0;

        public static final double JOYSTICK_MIN_THRESHOLD = 0.04;
    }

    public static class Autonomous {
        // Drivetrain characterization constants

        public static final double leftkS = CONST(0.367);
        public static final double leftkV = CONST(1.6);
        public static final double leftkA = CONST(0.0527);
        public static final double rightkS = CONST(0.361);
        public static final double rightkV = CONST(1.59);
        public static final double rightkA = CONST(0.0667);
        // Ramsete controller constants
        public static final double kBeta = 2;
        public static final double kZeta = 0.7;

        public static final double MAX_SPEED = 3; // [m/s]
        public static final double MAX_ACCELERATION = 2; // [m / s / s]
        public static final double MAX_CENTRIPETAL_ACCELERATION = 1.2; // [m / s / s]
    }

    public static class Vision {
        public static final double VISION_MODULE_HEIGHT = 0.98;
        public static final double VISION_MODULE_HOOD_DISTANCE = 0.28;
        public static final double VISION_ROTATION_RADIUS = 0.231; //The horizontal distance from the vision camera to the turret rotation axis.
        public static final double ROBOT_TO_TURRET_CENTER = 0.138; //The horizontal distance from the robot's center to the turret center.
    }

    public static class FieldGeometry {
        public static final Pose2d RED_OUTER_POWER_PORT_LOCATION = new Pose2d(15.98, 2.42, new Rotation2d());
        public static final Pose2d RED_INNER_POWER_PORT_LOCATION = new Pose2d(15.98 + 0.78, 2.42, new Rotation2d());
        public static final double OUTER_PORT_TO_LINE = 3.04;
//        public static final Pose2d BLUE_OUTER_POWER_PORT_LOCATION = new Pose2d(0, 5.79, new Rotation2d()); // The opponent location is (x: 0, y: 2.4).
//        public static final Pose2d BLUE_INNER_POWER_PORT_LOCATION = new Pose2d(-0.78, 5.79, new Rotation2d()); // The opponent location is (x: -0.78, y: 2.4).

        public static final double PORT_HEIGHT = 2.4;
    }

    public static final class Intake {
        public static final double INTAKE_POWER = 0.65;
        public static final double OUTTAKE_POWER = 0.2;
        public static final WebConstant PROPORTIONAL_INTAKE_VALUE = new WebConstant("intakeProportional", 0.165);
        public static final WebConstant INTAKE_CONSTANT_VALUE = new WebConstant("intakeConstant", 0.4);
    }

    public static class Conveyor {
        public static final double TICK_PER_METERS = 0.0382 * 4096;


        public static final WebConstant FEED_OUTTAKE_POWER = new WebConstant("feedOuttakePower", 0.6);
        public static final double PULSE_INTERVAL = 0.1;
        public static final double CONVEYOR_SMART_FEED_POWER = CONST(0.5);
        public static final double CONVEYOR_FEED_POWER = 0.7;
        public static final double FUNNEL_INTAKE_POWER = 0.25;
        public static final double CONVEYOR_INTAKE_POWER = 0.7;
        public static final double CONVEYOR_OUTTAKE_POWER = 0.5;
        public static final double FUNNEL_OUTTAKE_POWER = 0.6;
        public static final double OUTTAKE_TIME = 0.1;

        public static final double CONVEYOR_MOTOR_RETURN_POWER = 0;
        public static final double FEED_TIMEOUT = 5;

        public static final double INTAKE_PROXIMITY_MAX_VALUE = 200; //The minimum value for which the sensor would see a ball
        public static final double INTAKE_PROXIMITY_MIN_VALUE = 150; //The minimum voltage which the sensor would see in between two balls
        public static final double SHOOTER_PROXIMITY_MAX_VALUE = 2000;
        public static final double SHOOTER_PROXIMITY_MIN_VALUE = 800;

        public static final int MAX_BALLS_AMOUNT = 5;
        public static final int STARTING_AMOUNT = 3;

        public static final double GATE_OPEN_TIME = 0.5; // [sec] The amount of time from the opening of the gate until it is considered open

    }

    public static class Turret {
        public static final double VISION_TIMEOUT_SECONDS = 1;

        public static final int TICKS_PER_ROTATION = CONST(4096);
        public static final double TICKS_PER_DEGREE = CONST(TICKS_PER_ROTATION / 360.0);

        public static final DoubleRange ALLOWED_ANGLES = new DoubleRange(-42, 264);
        public static final DoubleRange DEAD_ZONE_ANGLES = new DoubleRange(38, 86);

        public static final double UNREACHABLE_ANGLE = 300; //This is an angle which the turret can't mechanically pass. If the turret passes this angle from either direction before startup, the turret will malfunction.
        public static final int ZERO_POSITION = CONST(1600); //Encoder absolute position when the turret is facing forward. This might change occasionally.

        public static final int POSITION_PID_SLOT = 0;
        public static final int MOTION_MAGIC_PID_SLOT = 2;

        public static double KP = CONST(3.5);
        public static double KI = CONST(0.01);
        public static double KD = CONST(180);
        public static double KF = CONST(0);

        public static double ALLOWABLE_ERROR = 0.3;

        public static double MOTION_MAGIC_KP = 4.07;
        public static double MOTION_MAGIC_KI = 0.0005;
        public static double MOTION_MAGIC_KD = 2.3;
        public static double MOTION_MAGIC_KF = 0;

        public static final int MOTION_MAGIC_CRUISE_VELOCITY = 3000;
        public static final int MOTION_MAGIC_ACCELERATION = 2000;

        public static final double DIRECT_VISION_KP = 0.01;
        public static final double DIRECT_VISION_KI = 0.001;
        public static final double DIRECT_VISION_KD = 0;

        public static final double TURRET_JOYSTICK_SPEED = 15; //Coefficient of the joystick value per degree.

        public static final int MAX_CURRENT = 30; // [A]
        public static final int PEAK_CURRENT = 0;
        public static final int PEAK_DURATION = 100;


        public static final double ANGLE_THRESHOLD = 1;
        public static final double VISION_ANGLE_THRESHOLD = 0.5;

        public static final int BACKLASH_ANGLE = 0; // The angle in which the motor moves without the mechanical system moving when switching direction
        public static final int VELOCITY_MINIMUM = 0; // Minimum velocity to indicate actual movement of the system instead of just small error

        public static final double CONTROL_MODE_THRESHOLD = 60;
    }


    public static class Shooter {
        public static final double TICKS_PER_ROTATION = 4096;

        public static final double KP = CONST(1);
        public static final double KI = CONST(0.0);
        public static final double KD = CONST(1.5);
        public static final double KF = CONST(0.014);

        public static DoubleRange ALLOWED_SHOOTING_RANGE = new DoubleRange(1, 10);

        public static final int MAX_CURRENT = 35; //[A]
        public static final double SHOOTING_TIME = 3.5; // [s]
        public static final double VELOCITY_TOLERANCE = 3; // [RPS] the acceptable velocity threshold error of the shooter
        public final static double MINIMAL_VELOCITY = 2;// [RPS] minimal velocity where the shooter knows it's actually moving
        public static final double VELOCITY_DAMP_RAMP =  1; // Damp ramp for that clamp on the accelerant
        public static final double VELOCITY_DAMPENING_LIMIT = 35; // Instead of trying to reach the target velocity, reach the current velocity + a constant.
    }

    public static class ColorWheel {
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

        /**
         * This is not a real PID control
         * Constants for a basic proportional control, when distance is 2 apply kP worth of percent output and apply kI when the distance is 1
         * The names are a meme please do not take kp and ki seriously 
         */
        public static double kP = 0.5; //Proportional constant for the control wheel Percent output when distance from the color is 2
        public static double kI = 0.3; //Proportional constant for the control panel Percent output when distance from the color is 1
        public static final double ROTATION_CONTROL_POWER = 0.4;
    }

    public static class Climber {
        public static final double TICKS_PER_METER = 4096 * 0.03 * Math.PI * 100; // TICKS * diameter * pi

        public static final double[] CLIMB_PIDF = {0.2, 0, 0, 0}; // Proportional, Integral, Derivative, Feedforward
        public static final double[] CLIMB_RELEASE_PIDF = {0.12, 0, 0, 0}; // Proportional, Integral, Derivative, Feedforward

        public static final double[] DELTA_PID = {0.0004, 0, 0}; // Proportional, Integral, Derivative

        public static final double MAX_HEIGHT = 0.75; // The allowed maximum height of the subsystem.
        public static final double HEIGHT_TARGET = 0.05;

        public static final double MIN_DELTA = 0.001;

        public static final double ARBITRARY_FEEDFORWARD = -0.35;

        public static final double RAMP_RATE = 0;

        public static final double CLIMB_HEIGHT = 0.65;
        public static final double SIMPLE_CLIMB_HEIGHT = 0.4;

        public static final double ALLOWED_HEIGHT_TOLERANCE = 0.05; // The allowed tolerance between the current height to the desired height.
        public static final double ALLOWED_ANGLE_TOLERANCE = 0.5; // The allowed tolerance between the current angle to the desired angle.
        public static final double MODIFY_JOYSTICK_RATE = 0.7; // The factor which the value of the joystick is multiplied by to calculate the change rate.
        public static final double MAX_DIFFERENCE = 2; // The maximal difference between the two sides of the climber.
        public static final double DISTANCE_BETWEEN_RODS = 0; // The distance between both climbing rods.
    }
}


//Anything in this class will replace the original constants when boolean is true
class BConstants {
    //General constants to be replaced

    public static final class Intake {

    }

    public static class Drivetrain {
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0.1;
        public static final double KF = 0;
    }

    public static class Autonomous {
        // Drivetrain characterization constants
        public static final double leftkS = 0.229;
        public static final double leftkV = 2.12;
        public static final double leftkA = 0.364;
        public static final double rightkS = 0.234;
        public static final double rightkV = 2.11;
        public static final double rightkA = 0.38;
    }

    public static class Turret {
        public static final double KD = 150;
        public static final int ZERO_POSITION = 645;
        public static final DoubleRange ALLOWED_ANGLES = new DoubleRange(-41, 227);
    }

    public static class Conveyor {
        public static final double CONVEYOR_FEED_POWER = 0.75;
    }

    public static class Shooter {
    }

}
