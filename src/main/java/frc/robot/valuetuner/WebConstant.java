package frc.robot.valuetuner;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import static spark.Spark.post;

/**
 * This class holds all the key value constants that will show up in the value tuner
 */
public class WebConstant {
    private static Map<String, ConstantObject> constantMap = new ConcurrentHashMap<>();
    private final ConstantObject constant;

    public WebConstant(String key, double value) {
        this.constant = new ConstantObject(key, value);
        constantMap.put(key, this.constant);
    }

    public double get() {
        return constant.getValue();
    }

    public static Map<String, ConstantObject> getConstantMap() {
        return constantMap;
    }

}
