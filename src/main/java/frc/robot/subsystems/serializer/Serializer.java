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
    private int ballsCount = 3;
    private double startLocation, endLocation;
    private boolean movingUp;
    private boolean entryBallInside, integrationBallInside, exitBallInside;

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
        if (isEntryProximityPressed() && movingUp && !entryBallInside) {
            incrementBallsCount(1);
            entryBallInside = true;
            startLocation = getEncoderPosition();
        } else
            entryBallInside = false;

        if (isExitProximityReleased() && movingUp && exitBallInside) {
            decrementBallsCount(1);
            exitBallInside = false;
            endLocation = getEncoderPosition();
        } else
            exitBallInside = true;

        if (isEntryProximityReleased() && !movingUp && entryBallInside) {
            decrementBallsCount(1);
            entryBallInside = false;
            startLocation = getEncoderPosition();
        } else
            entryBallInside = true;
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

    public boolean isEntryProximityPressed() {
        return entryProximity.getVoltage() > ENTRY_PROXIMITY_MAX_VOLTAGE;
    }

    public boolean isEntryProximityReleased() {
        return entryProximity.getVoltage() < ENTRY_PROXIMITY_MIN_VOLTAGE;
    }

    public boolean isIntegrationProximityPressed() {
        return integrationProximity.getVoltage() > INTEGRATION_PROXIMITY_MAX_VOLTAGE;
    }

    public boolean isIntegrationProximityReleased() {
        return integrationProximity.getVoltage() < INTEGRATION_PROXIMITY_MIN_VOLTAGE;
    }

    public boolean isExitProximityPressed() {
        return exitProximity.getVoltage() > EXIT_PROXIMITY_MAX_VOLTAGE;
    }

    public boolean isExitProximityReleased() {
        return exitProximity.getVoltage() < EXIT_PROXIMITY_MIN_VOLTAGE;
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

    public void setBallsCount(int ballsCount) {
        this.ballsCount = Math.max(0, Math.min(ballsCount, 5));
    }

    public void incrementBallsCount(int by) {
        ballsCount += by;
        setBallsCount(ballsCount);
    }

    public void decrementBallsCount(int by) {
        ballsCount -= by;
        setBallsCount(ballsCount);
    }

    public void feed() {
        exitMotor.set(ControlMode.PercentOutput, 60); //TODO choose reasonable value
        entryMotor.set(ControlMode.PercentOutput, 70);
    }
}