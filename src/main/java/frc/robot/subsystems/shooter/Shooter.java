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
    private UnitModel rpmUnitModel = new UnitModel(TICKS_PER_ROTATION);

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
    }

    /**
     * @return the speed of the shooter in rpm.
     */
    public double getSpeed() {
        return rpmUnitModel.toUnits(shooterMaster.getSelectedSensorVelocity() * 600);
    }

    /**
     * set the speed of the shooter.
     * @param rpm the rotations per minute of the shooter.
     */
    public void setSpeedRPM(double rpm) {
        shooterMaster.set(ControlMode.Velocity, rpmUnitModel.toTicks(rpm) / 600.); //convert rpm to ticks per 100ms
    }

}
