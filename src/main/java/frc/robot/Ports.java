package frc.robot;



public class Ports {
    public class climber {
        public static final int LEFT_MOTOR = 40;
        public static final int RIGHT_MOTOR = 41;
        public static final int STOPPER_FORWARD = 3;
        public static final int STOPPER_REVERSE = 0;
        public static final int STOPPER = 2;
        public static final boolean LEFT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_MOTOR_INVERTED = false;
        public static final boolean LEFT_ENCODER_INVERTED = false;
        public static final boolean RIGHT_ENCODER_INVERTED = false;
    }

    public static class Conveyor {
        public static final int MOTOR = 21;
        public static final boolean MOTOR_INVERTED = false;

        public static final int INTAKE_PROXIMITY = 0;
        public static final int SHOOTER_PROXIMITY = 1;

        public static final int FORWARD_GATE = 1;
        public static final int REVERSE_GATE = 2;
        public static final int GATE = 4; // Port of the stopper solenoid.
    }

    public static final int TALON_PID_SLOT = 0;

    public static final class Intake {
        public static final int MOTOR = 20;
        public static final boolean MOTOR_INVERTED = true;
        public static final boolean IS_FORWARD_OPEN = true; //if kForward of the solenoid opens the intake or reverse.
        public static final int FOLD_SOLENOID_REVERSE = 1;
        public static final int FOLD_SOLENOID_FORWARD = 2;
    }

    public static class Turret {
        public static final int MOTOR = 22;
        public static final boolean IS_MOTOR_INVERTED = false;
        public static final boolean IS_ENCODER_INVERTED = false;
    }

    public static class Shooter {
        public static final int MASTER = 23;
        public static final int SLAVE_1 = 24;
        public static final int SLAVE_2 = 25;
        public static final boolean IS_MASTER_INVERTED = false;
        public static final boolean IS_SLAVE_1_INVERTED = false;
        public static final boolean IS_SLAVE_2_INVERTED = false;
        public static final boolean IS_ENCODER_INVERTED = false;
    }

    public static class Drivetrain {
        public static final int LEFT_MASTER = 10;
        public static final int LEFT_SLAVE = 11;
        public static final int RIGHT_MASTER = 12;
        public static final int RIGHT_SLAVE = 13;

        public static final int SHIFTER_FORWARD_PORT = 1;
        public static final int SHIFTER_REVERSE_PORT = 2;
        public static final int SHIFTER_PORT = 1;
    }

    public static class ColorWheel {
        public static final int MOTOR = 30;
    }
}
