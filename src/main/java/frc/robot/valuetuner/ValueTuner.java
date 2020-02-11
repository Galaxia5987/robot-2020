package frc.robot.valuetuner;

import spark.ModelAndView;
import spark.template.velocity.VelocityTemplateEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static spark.Spark.get;
import static spark.Spark.post;

/**
 * The value tuner is a simple interface that allows you to change constants during run time.
 * This can be done by accessing the web interface located at http://10.59.87.2:5802 (roboRIO IP, port 5802).
 *
 * The tuner provides 2 main interface classes:
 *   {@link WebConstant}
 *   {@link WebConstant#get}
 *   {@link WebConstantPIDTalon}
 *   WebConstant is passed a key and a value
 *   WebConstantPIDTalon is passed a key, initial PIDSet and the talon you want to configure
 * Upon instantiation these interfaces will automatically configure themselves into the web interface
 *
 * Example usage:
 *  public static WebConstant vision_kP = new WebConstant("vision_kP");
 *  vision_Kp.get();
 *  new WebConstantPIDTalon("turretTalon", pidSet[0], pidSet[1], pidSet[2], pidSet[3], turretMaster);
 *
 *  Notes:
 *    - A value key should only be used once, hence it's recommended to use static when instantiating constants.
 */
public class ValueTuner {

    /**
     * Starts the value tuner web server.
     */
    public void start() {
        // Main route serving the tuner.vm template.
        get("/", (request, response) -> {
            Map<String, Object> model = new HashMap<>();
            List<ConstantObject> constants = new ArrayList<>(WebConstant.getConstantMap().values());
            List<TalonConstant> talons = new ArrayList<>(WebConstantPIDTalon.getConstantMap().values());
            model.put("constants", constants);
            model.put("talons", talons);
            return new VelocityTemplateEngine().render(
                    new ModelAndView(model, "tuner.vm")
            );
        });

        // Set route for normal constants.
        post("/set", (request, response) -> {
            String key = request.queryParamOrDefault("key", null);
            String value = request.queryParamOrDefault("value", null);
            ConstantObject constant = WebConstant.getConstantMap().get(key);
            if (constant != null) {
                constant.setValue(Double.parseDouble(value));
            }
            return "set";
        });

        // Set route for Talon constants.
        post("/setTalon", (request, response) -> {
            String key = request.queryParamOrDefault("key", null);
            String kP = request.queryParamOrDefault("kP", null);
            String kI = request.queryParamOrDefault("kI", null);
            String kD = request.queryParamOrDefault("kD", null);
            String kF = request.queryParamOrDefault("kF", null);
            TalonConstant talon = WebConstantPIDTalon.getConstantMap().get(key);
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

}
