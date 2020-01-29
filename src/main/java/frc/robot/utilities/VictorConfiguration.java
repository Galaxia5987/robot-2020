package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

public class VictorConfiguration {

    /**
     * This class is meant to be used as an effective way to give several VictorSPXs with the same
     * wanted configurations the same settings without a need to change each one separately
     * This uses the CTRE/WPILib VictorSPXConfiguration class and adds some likely needed parameters
     */

    private NeutralMode neutralMode;
    private boolean enableVoltageCompensation;
    private double voltageCompensationSaturation;
    public VictorSPXConfiguration motorConfigs = new VictorSPXConfiguration();


    public VictorConfiguration() {

        neutralMode = NeutralMode.Coast;
        enableVoltageCompensation = false;
        voltageCompensationSaturation = 12;


    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }


    public boolean isEnableVoltageCompensation() {
        return enableVoltageCompensation;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }


    public void setEnableVoltageCompensation(boolean enableVoltageCompensation) {
        this.enableVoltageCompensation = enableVoltageCompensation;
    }

    public double getVoltageCompensationSaturation() {
        return voltageCompensationSaturation;
    }

    public void setVoltageCompensationSaturation(double voltageCompensationSaturation) {
        this.voltageCompensationSaturation = voltageCompensationSaturation;
    }

    public void setPrimaryPID(VictorSPXPIDSetConfiguration primaryPID) {
        motorConfigs.primaryPID = primaryPID;
    }

    public void setAuxiliaryPID(VictorSPXPIDSetConfiguration auxiliaryPIDConfiguration) {
        motorConfigs.auxiliaryPID = auxiliaryPIDConfiguration;
    }

    public void setForwardLimitSwitchSource(RemoteLimitSwitchSource forwardLimitSwitchSource) {
        motorConfigs.forwardLimitSwitchSource = forwardLimitSwitchSource;
    }

    public void setReverseLimitSwitchSource(RemoteLimitSwitchSource reverseLimitSwitchSource) {
        motorConfigs.reverseLimitSwitchSource = reverseLimitSwitchSource;
    }

    public void setForwardLimitSwitchDeviceID(int forwardLimitSwitchDeviceID) {
        motorConfigs.forwardLimitSwitchDeviceID = forwardLimitSwitchDeviceID;
    }

    public void setReverseLimitSwitchDeviceID(int reverseLimitSwitchDeviceID) {
        motorConfigs.reverseLimitSwitchDeviceID = reverseLimitSwitchDeviceID;
    }

    public void setForwardLimitSwitchNormal(LimitSwitchNormal forwardLimitSwitchNormal) {
        motorConfigs.forwardLimitSwitchNormal = forwardLimitSwitchNormal;
    }

    public void setReverseLimitSwitchNormal(LimitSwitchNormal reverseLimitSwitchNormal) {
        motorConfigs.reverseLimitSwitchNormal = reverseLimitSwitchNormal;
    }

    public void setSum0Term(RemoteFeedbackDevice sum0Term) {
        motorConfigs.sum0Term = sum0Term;
    }

    public void setSum1Term(RemoteFeedbackDevice sum1Term) {
        motorConfigs.sum1Term = sum1Term;
    }

    public void setDiff0Term(RemoteFeedbackDevice diff0Term) {
        motorConfigs.diff0Term = diff0Term;
    }

    public void setDiff1Term(RemoteFeedbackDevice diff1Term) {
        motorConfigs.diff1Term = diff1Term;
    }

}
