package frc.robot;

public class Ports {


    public static class Turret {
        public static final int MOTOR = 41;
        public static final boolean IS_MOTOR_INVERTED = false;
        public static final boolean IS_ENCODER_INVERTED  = false;
    }
    public static class Shooter {
        public static final int MASTER = 0;
        public static final int SLAVE = 0;
        public static final boolean IS_MASTER_INVERTED = false;
        public static final boolean MASTER_SENSOR_PHASED = false;

    }
}
