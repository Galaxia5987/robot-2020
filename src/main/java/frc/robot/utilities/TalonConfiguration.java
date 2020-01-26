package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

public class TalonConfiguration {

    /**
     * This class is meant to be used as an effective way to give several talons with the same
     * wanted configurations the same settings without a need to change each one separately
     * This uses the CTRE/WPILib TalonSRXConfiguration class and adds some likely needed parameters
     */

    private NeutralMode neutralMode;
    private FeedbackDevice feedbackDevice;
    private boolean enableVoltageCompensation;
    private boolean enableCurrentLimit;
    private int continuousCurrentLimit = 0;
    private int peakCurrentLimit = 0;
    public TalonSRXConfiguration motorConfigs = new TalonSRXConfiguration();


    public TalonConfiguration() {

        neutralMode = NeutralMode.Coast;
        feedbackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;
        enableVoltageCompensation = false;


    }

    public boolean isEnableCurrentLimit() {
        return enableCurrentLimit;
    }

    public void setEnableCurrentLimit(boolean enableCurrentLimit) {
        this.enableCurrentLimit = enableCurrentLimit;
    }

    public int getContinuousCurrentLimit() {
        return continuousCurrentLimit;
    }

    public void setContinuousCurrentLimit(int continuousCurrentLimit) {
        this.continuousCurrentLimit = continuousCurrentLimit;
    }

    public int getPeakCurrentLimit() {
        return peakCurrentLimit;
    }

    public void setPeakCurrentLimit(int peakCurrentLimit) {
        this.peakCurrentLimit = peakCurrentLimit;
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

    public void setPrimaryPID(TalonSRXPIDSetConfiguration primaryPIDConfiguration) {
        this.motorConfigs.primaryPID = primaryPIDConfiguration;
    }

    public void setAuxiliaryPID(TalonSRXPIDSetConfiguration auxiliaryPID) {
        motorConfigs.auxiliaryPID = auxiliaryPID;
    }

    public void setForwardLimitSwitchSource(LimitSwitchSource forwardLimitSwitchSource) {
        motorConfigs.forwardLimitSwitchSource = forwardLimitSwitchSource;
    }

    public void setReverseLimitSwitchSource(LimitSwitchSource reverseLimitSwitchSource) {
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

    public void setSum0Term(FeedbackDevice sum0Term) {
        motorConfigs.sum0Term = sum0Term;
    }

    public void setSum1Term(FeedbackDevice sum1Term) {
        motorConfigs.sum1Term = sum1Term;
    }

    public void setDiff0Term(FeedbackDevice diff0Term) {
        motorConfigs.diff0Term = diff0Term;
    }

    public void setDiff1Term(FeedbackDevice diff1Term) {
        motorConfigs.diff1Term = diff1Term;
    }

}
