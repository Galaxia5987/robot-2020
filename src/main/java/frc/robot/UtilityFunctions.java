package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.utilities.FalconConfiguration;

public class UtilityFunctions {
    public static void configAllFalcons(FalconConfiguration configurations, TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            falcon.configAllSettings(configurations.motorConfigs);
            falcon.setNeutralMode(configurations.getNeutralMode());
            falcon.enableVoltageCompensation(configurations.isEnableVoltageCompensation());
            falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(configurations.isEnableCurrentLimit()
                    , configurations.getSupplyCurrentLimit()
                    , configurations.getThreshHoldCurrent()
                        , configurations.getThreshHoldTime()));
            falcon.config_kP(0, configurations.getPidSet()[0]);
            falcon.config_kI(0, configurations.getPidSet()[1]);
            falcon.config_kD(0, configurations.getPidSet()[2]);
            falcon.config_kF(0, configurations.getPidSet()[3]);

        }
    }
}
