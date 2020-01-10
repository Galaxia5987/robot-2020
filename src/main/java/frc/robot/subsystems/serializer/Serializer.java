package frc.robot.subsystems.serializer;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Serializer.*;
import static frc.robot.Ports.Serializer.*;

public class Serializer extends SubsystemBase {
    private TalonSRX serializerExitMotor = new TalonSRX(EXIT_MOTOR);
    private VictorSPX serializerEntryMotor = new VictorSPX(ENTRY_MOTOR);
    private AnalogInput entryProximity = new AnalogInput(ENTRY_PROXIMITY);
    private AnalogInput middleProximity = new AnalogInput(MIDDLE_PROXIMITY);
    private AnalogInput exitProximity = new AnalogInput(EXIT_PROXIMITY);

    public Serializer() {
        serializerExitMotor.configFactoryDefault();
        serializerEntryMotor.configFactoryDefault();

        serializerExitMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TALON_PID_SLOT, TALON_TIMEOUT_MS);
        serializerExitMotor.setSensorPhase(EXIT_PHASED);
        serializerEntryMotor.setSensorPhase(ENTRY_PHASED);
        serializerExitMotor.setInverted(EXIT_INVERTED);
        serializerEntryMotor.setInverted(ENTRY_INVERTED);

        serializerExitMotor.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT_MS);
        serializerExitMotor.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT_MS);
        serializerExitMotor.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT_MS);

        serializerExitMotor.configMotionCruiseVelocity(CRUISE_VELOCITY);
        serializerExitMotor.configMotionAcceleration(CRUISE_ACCELERATION, TALON_TIMEOUT_MS);
        serializerExitMotor.configPeakCurrentLimit(MAX_CURRENT);
        serializerExitMotor.configClosedloopRamp(RAMP_RATE);

        serializerExitMotor.configVoltageCompSaturation(12);
        serializerEntryMotor.configVoltageCompSaturation(12);
        serializerExitMotor.enableVoltageCompensation(true);
        serializerEntryMotor.enableVoltageCompensation(true);
        serializerExitMotor.setSelectedSensorPosition(0);
    }
}
