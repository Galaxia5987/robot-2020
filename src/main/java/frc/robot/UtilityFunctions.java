package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class UtilityFunctions {
    public static void configAllFalcons(FalconFiguration configs, TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            falcon.configAllSettings(configs.motorConfigs);
            falcon.setNeutralMode(NeutralMode.Brake);
            falcon.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            falcon.enableVoltageCompensation(configs.isEnableVoltageCompensation());

        }
    }
}
