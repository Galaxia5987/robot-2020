package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
}
