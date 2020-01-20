package frc.robot.valuetuner;

import spark.ModelAndView;
import spark.template.velocity.VelocityTemplateEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static spark.Spark.get;
import static spark.Spark.port;

public class ValueTuner {

    public void start() {
        port(5802);
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
    }

}
