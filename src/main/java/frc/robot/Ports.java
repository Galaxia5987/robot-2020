package frc.robot;

public class Ports {
    public static final int TALON_PID_SLOT = 0;


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
