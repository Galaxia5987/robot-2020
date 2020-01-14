package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.utilities.VictorConfiguration;
import frc.robot.utilities.TalonConfiguration;

public class UtilityFunctions {

    public static void configAllTalons(TalonConfiguration configs, TalonSRX... talons) {
        for (TalonSRX talon : talons) {
            talon.configAllSettings(configs.motorConfigs);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            talon.enableVoltageCompensation(configs.isEnableVoltageCompensation());
            talon.enableCurrentLimit(configs.isEnableCurrentLimit());
            talon.configPeakCurrentLimit(configs.getPeakCurrentLimit());

        }

    }

    public static void configAllVictors(VictorConfiguration configs, VictorSPX... victors) {
        for (VictorSPX victor : victors) {
            victor.configAllSettings(configs.motorConfigs);
            victor.setNeutralMode(configs.getNeutralMode());
            victor.configSelectedFeedbackSensor(configs.getFeedbackDevice());
            victor.enableVoltageCompensation(configs.isEnableVoltageCompensation());


        }
    }
}
