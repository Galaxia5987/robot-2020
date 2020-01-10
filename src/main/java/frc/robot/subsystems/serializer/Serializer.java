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
    private double startLocation, endLocation;
    private boolean movingUp;

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
        if (getEntryProximity() && movingUp) {
            incrementBalls(1);
            startLocation = getEncoderPosition();
        }
        if (getExitProximity() && movingUp) {
            decrementBalls(1);
            endLocation = getEncoderPosition();
        }
    }

    public int getEntryVelocity() {
        return entryMotor.getSelectedSensorVelocity();
    }

    public void setEntryVelocity(double speed) {
        movingUp = (speed >= 0);
        entryMotor.set(ControlMode.Velocity, speed);
    }

    public int getExitVelocity() {
        return exitMotor.getSelectedSensorVelocity();
    }

    public void setExitVelocity(double location) {
        movingUp = (location >= 0);
        exitMotor.set(ControlMode.MotionMagic, location);
    }

    public boolean getEntryProximity() {
        return entryProximity.getVoltage() > ENTRY_PROXIMITY_VOLTAGE;
    }

    public boolean getIntegrationProximity() {
        return integrationProximity.getVoltage() > INTEGRATION_PROXIMITY_VOLTAGE;
    }

    public boolean getExitProximity() {
        return exitProximity.getVoltage() > EXIT_PROXIMITY_VOLTAGE;
    }

    public void moveConveyor(double location, double speed) {
        setExitVelocity(getEncoderPosition() + location);
        setEntryVelocity(speed);
    }

    public void moveConveyor(double location) {
        moveConveyor(location, 0.5); //TODO Change the speed to reasonable value
    }

    public double getEncoderPosition() {
        return model.toUnits(exitMotor.getSelectedSensorPosition());
    }

    public void maximizeConveyor() {
        moveConveyor(endLocation, 0.5); //TODO choose real number
    }

    public void minimizeConveyor() {
        moveConveyor(startLocation, -0.1); //TODO choose real number
    }

    public void setBalls(int balls) {
        this.balls = Math.max(0, Math.min(balls, 5));
    }

    public void incrementBalls(int by) {
        balls += by;
        setBalls(balls);
    }

    public void decrementBalls(int by) {
        balls-= by;
        setBalls(balls);
    }
}

