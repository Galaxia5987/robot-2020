package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;

public class VictorConfiguration {

    /**
     * This class is meant to be used as an effective way to give several victors with the same
     * wanted configurations the same settings without a need to change each one separately
     * This uses the wpilib VictorSPXConfiguration class and adds some likely needed parameters
     */

    private NeutralMode neutralMode;
    private boolean enableVoltageCompensation;
    public VictorSPXConfiguration motorConfigs = new VictorSPXConfiguration();


    public VictorConfiguration() {

        neutralMode = NeutralMode.Coast;
        enableVoltageCompensation = false;


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

    public void setPrimaryPID(VictorSPXPIDSetConfiguration primaryPID) {
        this.motorConfigs.primaryPID = primaryPID;
    }

    public void setAuxiliaryPID(VictorSPXPIDSetConfiguration auxiliaryPID) {
        this.motorConfigs.auxiliaryPID = auxiliaryPID;
    }

    public void setForwardLimitSwitchSource(RemoteLimitSwitchSource forwardLimitSwitchSource) {
        this.motorConfigs.forwardLimitSwitchSource = forwardLimitSwitchSource;
    }

    public void setReverseLimitSwitchSource(RemoteLimitSwitchSource reverseLimitSwitchSource) {
        this.motorConfigs.reverseLimitSwitchSource = reverseLimitSwitchSource;
    }

    public void setForwardLimitSwitchDeviceID(int forwardLimitSwitchDeviceID) {
        this.motorConfigs.forwardLimitSwitchDeviceID = forwardLimitSwitchDeviceID;
    }

    public void setReverseLimitSwitchDeviceID(int reverseLimitSwitchDeviceID) {
        this.motorConfigs.reverseLimitSwitchDeviceID = reverseLimitSwitchDeviceID;
    }

    public void setForwardLimitSwitchNormal(LimitSwitchNormal forwardLimitSwitchNormal) {
        this.motorConfigs.forwardLimitSwitchNormal = forwardLimitSwitchNormal;
    }

    public void setReverseLimitSwitchNormal(LimitSwitchNormal reverseLimitSwitchNormal) {
        this.motorConfigs.reverseLimitSwitchNormal = reverseLimitSwitchNormal;
    }

    public void setSum0Term(RemoteFeedbackDevice sum0Term) {
        this.motorConfigs.sum0Term = sum0Term;
    }

    public void setSum1Term(RemoteFeedbackDevice sum1Term) {
        this.motorConfigs.sum1Term = sum1Term;
    }

    public void setDiff0Term(RemoteFeedbackDevice diff0Term) {
        this.motorConfigs.diff0Term = diff0Term;
    }

    public void setDiff1Term(RemoteFeedbackDevice diff1Term) {
        this.motorConfigs.diff1Term = diff1Term;
    }

}