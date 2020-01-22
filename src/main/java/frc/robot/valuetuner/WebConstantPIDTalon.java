package frc.robot.valuetuner;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import static spark.Spark.post;

public class WebConstantPIDTalon {
    private static Map<String, TalonConstant> constantMap = new ConcurrentHashMap<>();

    public WebConstantPIDTalon(String key, double kP, double kI, double kD, double kF, BaseTalon talon) {
        constantMap.put(key, new TalonConstant(key, kP, kI, kD, kF, talon));
    }

    static {
        post("/setTalon", (request, response) -> {
            String key = request.queryParamOrDefault("key", null);
            String kP = request.queryParamOrDefault("kP", null);
            String kI = request.queryParamOrDefault("kI", null);
            String kD = request.queryParamOrDefault("kD", null);
            String kF = request.queryParamOrDefault("kF", null);
            TalonConstant talon = constantMap.get(key);
            if (kP != null) {
                talon.getTalon().config_kP(0, Double.parseDouble(kP));
            }
            if (kI != null) {
                talon.getTalon().config_kI(0, Double.parseDouble(kI));
            }
            if (kD != null) {
                talon.getTalon().config_kD(0, Double.parseDouble(kD));
            }
            if (kF != null) {
                talon.getTalon().config_kF(0, Double.parseDouble(kF));
            }
            return "set";
        });
    }

    public static Map<String, TalonConstant> getConstantMap() {
        return constantMap;
    }

}
