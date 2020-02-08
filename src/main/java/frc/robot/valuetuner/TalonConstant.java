package frc.robot.valuetuner;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * This class holds kP, kI, kD, kF to be configured on a Talon (SRX/FX) motor controller provided through the constructor.
 * These values are held behind a key.
 */
public class TalonConstant {
    private final String key;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final BaseTalon talon;

    TalonConstant(String key, double kP, double kI, double kD, double kF, BaseTalon talon) {
        this.key = key;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.talon = talon;
    }

    public BaseTalon getTalon() {
        return talon;
    }

    public String getKey() {
        return key;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
        return kF;
    }
}
