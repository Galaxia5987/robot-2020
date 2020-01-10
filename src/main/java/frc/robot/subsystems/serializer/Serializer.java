package frc.robot.subsystems.serializer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Serializer.*;
import static frc.robot.Ports.Serializer.*;

public class Serializer extends SubsystemBase {
    UnitModel model = new UnitModel(TICK_PER_METERS);
    private TalonSRX exitMotor = new TalonSRX(EXIT_MOTOR);
    private VictorSPX entryMotor = new VictorSPX(ENTRY_MOTOR);
    private AnalogInput entryProximity = new AnalogInput(ENTRY_PROXIMITY);
    private AnalogInput integrationProximity = new AnalogInput(MIDDLE_PROXIMITY);
    private AnalogInput exitProximity = new AnalogInput(EXIT_PROXIMITY);
    private int balls = 3;

    public Serializer() {
        exitMotor.configFactoryDefault();
        entryMotor.configFactoryDefault();

        exitMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, TALON_TIMEOUT_MS);
        exitMotor.setSensorPhase(EXIT_PHASED);
        entryMotor.setSensorPhase(ENTRY_PHASED);
        exitMotor.setInverted(EXIT_INVERTED);
        entryMotor.setInverted(ENTRY_INVERTED);

        exitMotor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT_MS);
        exitMotor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT_MS);
        exitMotor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT_MS);

        exitMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        exitMotor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT_MS);
        exitMotor.configPeakCurrentLimit(MAX_CURRENT);
        exitMotor.configClosedloopRamp(RAMP_RATE);

        exitMotor.configVoltageCompSaturation(12);
        entryMotor.configVoltageCompSaturation(12);
        exitMotor.enableVoltageCompensation(true);
        entryMotor.enableVoltageCompensation(true);
        exitMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (getEntryProximity())
            balls++;
        if (getExitProximity())
            balls--;

    }

    public void setEntrySpeed(double speed) {
        entryMotor.set(ControlMode.Velocity, speed);
    }

    public void setExitSpeed(double speed) {
        exitMotor.set(ControlMode.MotionMagic, speed);
    }

    public boolean getEntryProximity(){
        return entryProximity.getVoltage() > ENTRY_PROXIMITY_VOLTAGE;
    }

    public boolean getIntegrationProximity(){
        return integrationProximity.getVoltage() > INTEGRATION_PROXIMITY_VOLTAGE;
    }

    public boolean getExitProximity(){
        return exitProximity.getVoltage() > EXIT_PROXIMITY_VOLTAGE;
    }
    public double getEncoderPosition() {
        return model.toUnits(exitMotor.getSelectedSensorPosition());
    }

    public void maximizeConveyor() {
        while (!getExitProximity()) {
            moveConveyor(,0.5); //TODO choose real number
        }
    }

    public void minimizeConveyor() {
        while (!getEntryProximity()) {
            moveConveyor(,-0.1); //TODO choose real number
        }
    }
}

