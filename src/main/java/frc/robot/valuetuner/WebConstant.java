package frc.robot.valuetuner;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import static spark.Spark.post;

public class WebConstant {
    private static Map<String, ConstantObject> constantMap = new ConcurrentHashMap<>();
    private final ConstantObject constant;

    static {
        post("/set", (request, response) -> {
            String key = request.queryParamOrDefault("key", null);
            String value = request.queryParamOrDefault("value", null);
            ConstantObject constant = WebConstant.getConstantMap().get(key);
            if (constant != null) {
                constant.setValue(Double.parseDouble(value));
            }
            return "set";
        });
    }


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
