package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.utilities.FalconConfiguration;
import frc.robot.utilities.VictorConfiguration;
import frc.robot.utilities.TalonConfiguration;

public class UtilityFunctions {

    /**
     * This method uses the custom TalonConfiguration class to config all given talons
     * @param configs The custom talon configuration instance
     * @param talons All talons
     */
    public static void configAllTalons(TalonConfiguration configs, TalonSRX... talons) {
        for (TalonSRX talon : talons) {
            talon.configAllSettings(configs.motorConfigs);
            talon.setNeutralMode(configs.getNeutralMode());
            talon.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            talon.enableVoltageCompensation(configs.isEnableVoltageCompensation());
            talon.enableCurrentLimit(configs.isEnableCurrentLimit());
            talon.configContinuousCurrentLimit(configs.getContinuousCurrentLimit());
            talon.configPeakCurrentLimit(configs.getPeakCurrentLimit());
        }

    }

    /**
     * This method uses the custom VictorConfiguration class to config all given victors
     * @param configs The custom victor configuration instance
     * @param victors All victors
     */
    public static void configAllVictors(VictorConfiguration configs, VictorSPX... victors) {
        for (VictorSPX victor : victors) {
            victor.configAllSettings(configs.motorConfigs);
            victor.setNeutralMode(configs.getNeutralMode());
            victor.enableVoltageCompensation(configs.isEnableVoltageCompensation());


        }
    }

    public static void configAllFalcons(FalconConfiguration configurations, TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            falcon.enableVoltageCompensation(configurations.isEnableVoltageCompensation());
            falcon.setNeutralMode(configurations.getNeutralMode());
            falcon.configAllSettings(configurations.motorConfigs);
            falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(configurations.isEnableCurrentLimit()
                    , configurations.getSupplyCurrentLimit()
                    , configurations.getThreshHoldCurrent()
                    , configurations.getThreshHoldTime()));
            falcon.configVoltageCompSaturation(configurations.getVoltageCompensationSaturation());
            falcon.config_kP(0, configurations.getPidSet()[0]);
            falcon.config_kI(0, configurations.getPidSet()[1]);
            falcon.config_kD(0, configurations.getPidSet()[2]);
            falcon.config_kF(0, configurations.getPidSet()[3]);

        }
    }
}