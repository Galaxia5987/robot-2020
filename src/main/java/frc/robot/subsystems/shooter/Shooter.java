package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase {
    private TalonSRX shooterMaster = new TalonSRX(MASTER);
    private VictorSPX shooterSlave = new VictorSPX(SLAVE);
    private VictorSPX inputMotor = new VictorSPX(INPUT_MOTOR);
    private UnitModel unitModel = new UnitModel(TICKS_PER_ROTATION);

    public Shooter() {
        shooterMaster.configFactoryDefault();
        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        shooterSlave.follow(shooterMaster);
        shooterMaster.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        shooterMaster.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        shooterMaster.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        shooterMaster.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);
        shooterMaster.setInverted(IS_MASTER_INVERTED);
        shooterMaster.setSensorPhase(MASTER_SENSOR_PHASED);
        shooterMaster.configClosedloopRamp(RAMP_RATE);
        shooterMaster.configVoltageCompSaturation(12);
        shooterMaster.enableVoltageCompensation(true);
        shooterSlave.setInverted(IS_SLAVE_INVERTED);
        shooterSlave.setSensorPhase(SLAVE_SENSOR_PHASED);
        shooterSlave.configVoltageCompSaturation(12);
        shooterSlave.enableVoltageCompensation(true);
        shooterMaster.configPeakCurrentLimit(MAX_CURRENT);
        shooterMaster.setSelectedSensorPosition(0);
        // TODO: Configure peak and nominal outputs, if needed
    }

    /**
     * @return the speed of the shooter in rpm.
     */
    public double getSpeed() {
        return ticksToRPM(shooterMaster.getSelectedSensorVelocity());
    }

    /**
     * set the speed of the shooter.
     * @param rpm the rotations per minute of the shooter.
     */
    public void setSpeedRPM(double rpm) {
        shooterMaster.set(ControlMode.Velocity, ticksToRPM(rpm));
    }

    /**
     * @param ticks the encoder units of the Talon.
     * @return the conversion between ticks to rpm.
     */
    public double ticksToRPM(double ticks) {
        return unitModel.toUnits(ticks) / 10.0 / 60.0; // TODO: fix conversions and add gear ratio
    }

    /**
     * @param rpm the rotations per minute of the shooter.
     * @return the conversion between rpm to ticks.
     */
    public double RPMToTicks(double rpm) {
        return unitModel.toTicks(rpm) * 10.0 * 60.0;
    }
}
