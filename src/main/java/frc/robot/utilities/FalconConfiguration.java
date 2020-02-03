package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;

public class FalconConfiguration {
    private NeutralMode neutralMode;
    private boolean enableVoltageCompensation;
    private double voltageCompensationSaturation;
    private double supplyCurrentLimit = 0;
    private boolean enableCurrentLimit = false;
    private double threshHoldCurrent = 0;
    private double threshHoldTime = 0;

    private double[] pidSet = {0, 0, 0, 0};
    public TalonFXConfiguration motorConfigs = new TalonFXConfiguration();


    public FalconConfiguration() {

        neutralMode = NeutralMode.Coast;
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

    public boolean isEnableVoltageCompensation() {
        return enableVoltageCompensation;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public void setEnableVoltageCompensation(boolean enableVoltageCompensation) {
        this.enableVoltageCompensation = enableVoltageCompensation;
    }

    public void configureVoltageCompensationSaturation(double volts) {
        voltageCompensationSaturation = volts;
    }

    public void setPrimaryPID(TalonFXPIDSetConfiguration primaryPID) {
        this.motorConfigs.primaryPID = primaryPID;
    }

    public void setAuxiliaryPID(TalonFXPIDSetConfiguration auxiliaryPID) {
        this.motorConfigs.auxiliaryPID = auxiliaryPID;
    }

    public void setPGain(double pGain){
        this.pidSet[0] = pGain;
    }
    public void setIGain(double iGain){
        this.pidSet[1] = iGain;
    }
    public void setDGain(double dGain){
        this.pidSet[2] = dGain;
    }
    public void setFGain(double fGain){
        this.pidSet[3] = fGain;
    }
    public void setPidSet(double pGain, double iGain, double dGain, double fGain){
        setPGain(pGain);
        setIGain(iGain);
        setDGain(dGain);
        setFGain(fGain);
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

    public double getVoltageCompensationSaturation() {
        return voltageCompensationSaturation;
    }


}
