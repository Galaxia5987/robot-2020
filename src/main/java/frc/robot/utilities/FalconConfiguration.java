package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;

public class FalconConfiguration {
    private NeutralMode neutralMode;
    private FeedbackDevice feedbackDevice;
    private boolean enableVoltageCompensation;
    private double supplyCurrentLimit = 0;
    private boolean enableCurrentLimit = false;
    private double threshHoldCurrent = 0;
    private double threshHoldTime = 0;



    private double[] pidSet = {0, 0, 0, 0};
    public TalonFXConfiguration motorConfigs = new TalonFXConfiguration();


    public FalconConfiguration() {

        neutralMode = NeutralMode.Coast;
        feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Absolute;
        enableVoltageCompensation = false;


    }

    public double getSupplyCurrentLimit() {
        return supplyCurrentLimit;
    }

    public void setSupplyCurrentLimit(double supplyCurrentLimit) {
        this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public boolean isEnableCurrentLimit() {
        return enableCurrentLimit;
    }

    public void setEnableCurrentLimit(boolean enableCurrentLimit) {
        this.enableCurrentLimit = enableCurrentLimit;
    }

    public double getThreshHoldCurrent() {
        return threshHoldCurrent;
    }

    public void setThreshHoldCurrent(double threshHoldCurrent) {
        this.threshHoldCurrent = threshHoldCurrent;
    }

    public double getThreshHoldTime() {
        return threshHoldTime;
    }

    public void setThreshHoldTime(double threshHoldTime) {
        this.threshHoldTime = threshHoldTime;
    }

    public NeutralMode getNeutralMode() {
        return this.neutralMode;
    }

    public FeedbackDevice getFeedbackDevice() {
        return feedbackDevice;
    }

    public boolean isEnableVoltageCompensation() {
        return enableVoltageCompensation;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    public void setEnableVoltageCompensation(boolean enableVoltageCompensation) {
        this.enableVoltageCompensation = enableVoltageCompensation;
    }

    public void setPrimaryPID(TalonFXPIDSetConfiguration primaryPID) {
        this.motorConfigs.primaryPID = primaryPID;
    }

    public void setAuxiliaryPID(TalonFXPIDSetConfiguration auxiliaryPID) {
        this.motorConfigs.auxiliaryPID = auxiliaryPID;
    }

    public void setPidSet(double[] pidSet) {
        this.pidSet = pidSet;
    }

    public double[] getPidSet() {
        return pidSet;
    }

    public void setForwardLimitSwitchSource(LimitSwitchSource forwardLimitSwitchSource) {
        this.motorConfigs.forwardLimitSwitchSource = forwardLimitSwitchSource;
    }

    public void setReverseLimitSwitchSource(LimitSwitchSource reverseLimitSwitchSource) {
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

    public void setSum0Term(FeedbackDevice sum0Term) {
        this.motorConfigs.sum0Term = sum0Term;
    }

    public void setSum1Term(FeedbackDevice sum1Term) {
        this.motorConfigs.sum1Term = sum1Term;
    }

    public void setDiff0Term(FeedbackDevice diff0Term) {
        this.motorConfigs.diff0Term = diff0Term;
    }

    public void setDiff1Term(FeedbackDevice diff1Term) {
        this.motorConfigs.diff1Term = diff1Term;
    }
}
