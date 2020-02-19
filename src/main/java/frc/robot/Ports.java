package frc.robot;

import static org.apache.commons.lang3.ObjectUtils.CONST;

@SuppressWarnings("ConstantConditions")
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
        public static final int GATE = 0; // Port of the stopper solenoid.
        public static final boolean IS_GATE_REVERSED = CONST(false);
    }

    public static final class Intake {
        public static final int MOTOR = 20;
        public static final boolean MOTOR_INVERTED = CONST(true);
        public static final boolean IS_FORWARD_OPEN = CONST(true); //if kForward of the solenoid opens the intake or reverse.
        public static final int FOLD_SOLENOID_REVERSE = 7;
        public static final int FOLD_SOLENOID_FORWARD = 6;
        public static final int SOLENOID = 2;
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
        public static final int SHIFTER_PORT = 1;
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
}

class BPorts{
    public static class Shooter {
        public static final boolean IS_MASTER_INVERTED = true;
        public static final boolean IS_SLAVE_1_INVERTED = false;
        public static final boolean IS_SLAVE_2_INVERTED = true;
        public static final boolean IS_ENCODER_INVERTED = false;
    }


    public static class Turret {
        public static final boolean IS_MOTOR_INVERTED = true;
        public static final boolean IS_ENCODER_INVERTED = true;

        public static final boolean ENABLE_SOFT_LIMITS = true;
        public static final boolean DISABLE_SOFT_LIMITS_ON_DISCONNECT = true;
    }

    public static class Drivetrain {
    }

    public static class ColorWheel {
    }

    public static class Conveyor {
        public static final boolean FUNNEL_INVERTED = false;
        public static final boolean MOTOR_INVERTED = true;
    }

    public static class Climber {
    }
}

