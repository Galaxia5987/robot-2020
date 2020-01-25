package frc.robot;


public class Ports {
    public static class Conveyor {
        public static final int MOTOR = 21;
        public static final boolean MOTOR_INVERTED = false;

        public static final int INTAKE_PROXIMITY = 0;
        public static final int SHOOTER_PROXIMITY = 1;

        public static final int GATE = 4; //Port of the stopper solenoid
    }

    public static final int TALON_PID_SLOT = 0;

    public static final class Intake {
        public static final int MOTOR = 20;
        public static final boolean MOTOR_INVERTED = true;
        public static final boolean IS_FORWARD_OPEN = true; //if kForward of the solenoid opens the intake or reverse.
        public static final int FOLD_SOLENOID_REVERSE = 1;
        public static final int FOLD_SOLENOID_FORWARD = 2;
        ;
    }

    public static class Turret {
        public static final int MOTOR = 22;
        public static final boolean IS_MOTOR_INVERTED = false;
        public static final boolean IS_ENCODER_INVERTED  = false;
    }
    public static class Shooter {
        public static final int MASTER = 23;
        public static final int SLAVE_1 = 24;
        public static final int SLAVE_2 = 25;
        public static final boolean IS_MASTER_INVERTED = false;
        public static final boolean IS_SLAVE_1_INVERTED = false;
        public static final boolean IS_SLAVE_2_INVERTED = false;
        public static final boolean IS_MASTER_ENCODER_INVERTED = false;
    }
}
