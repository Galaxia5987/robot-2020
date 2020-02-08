package frc.robot.valuetuner;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import static spark.Spark.post;

/**
 * This class holds all the Talon PID constants that will show up in the value tuner.
 */
public class WebConstantPIDTalon {
    private static Map<String, TalonConstant> constantMap = new ConcurrentHashMap<>();

    public WebConstantPIDTalon(String key, double kP, double kI, double kD, double kF, BaseTalon talon) {
        constantMap.put(key, new TalonConstant(key, kP, kI, kD, kF, talon));
    }

    public static Map<String, TalonConstant> getConstantMap() {
        return constantMap;
    }

}
