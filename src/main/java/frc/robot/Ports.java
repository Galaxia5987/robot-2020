package frc.robot;


public class Ports {
    public static class Conveyor {
        public static final int MOTOR = 22;

        public static final int INTAKE_PROXIMITY = 0; //Bottom proximity
        public static final int CONVEYOR_PROXIMITY = 0; //Top proximity

        public static final int GATE = 0;
    }

    public static final int TALON_PID_SLOT = 0;

    public static final class Intake {
        public static final int MASTER = 11;
        //TODO: Check, may need to switch between the channels
        public static final int FOLD_SOLENOID_FORWARD = 0;
        public static final int FOLD_SOLENOID_REVERSE = 0;
    }    

    public static class Turret {
        public static final int MOTOR = 41;
        public static final boolean IS_MOTOR_INVERTED = false;
        public static final boolean IS_ENCODER_INVERTED  = false;
    }
    public static class Shooter {
        public static final int MASTER = 51;
        public static final int SLAVE = 52;
        public static final boolean IS_MASTER_INVERTED = false;
        public static final boolean IS_MASTER_ENCODER_INVERTED = false;
        public static final boolean IS_SLAVE_INVERTED = false;
    }
}
