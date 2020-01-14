package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.TALON_TIMEOUT;
import static frc.robot.Ports.Shooter.*;
import static frc.robot.Ports.TALON_PID_SLOT;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterMaster = new TalonSRX(MASTER);
    private final UnitModel rpsUnitModel = new UnitModel(TICKS_PER_ROTATION);

    public Shooter() {
        // Basic motor configurations
        // Master configurations
        shooterMaster.configFactoryDefault();
        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        shooterMaster.setInverted(IS_MASTER_INVERTED);
        shooterMaster.setSensorPhase(IS_MASTER_ENCODER_INVERTED);

        // Closed loop control
        shooterMaster.configClosedloopRamp(MAX_ACCELERATION);
        shooterMaster.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        shooterMaster.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        shooterMaster.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        shooterMaster.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);

        // Electrical (master)
        shooterMaster.configVoltageCompSaturation(12);
        shooterMaster.enableVoltageCompensation(true);
        shooterMaster.configPeakCurrentLimit(MAX_CURRENT);

        // Slave configuration
        VictorSPX shooterSlave = new VictorSPX(SLAVE);
        shooterSlave.follow(shooterMaster);
        shooterMaster.setInverted(IS_SLAVE_INVERTED);
        shooterMaster.setSensorPhase(IS_SLAVE_ENCODER_INVERTED);

        // Electrical (slave)
        shooterSlave.configVoltageCompSaturation(12);
        shooterSlave.enableVoltageCompensation(true);
    }

    /**
     * @return the speed of the shooter in rpm.
     */
    public double getSpeed() {
        return rpsUnitModel.toUnits(shooterMaster.getSelectedSensorVelocity() * 10);
    }

    /**
     * set the speed of the shooter.
     * @param speed the rotations per minute of the shooter.
     */
    public void setSpeed(double speed) {
        shooterMaster.set(ControlMode.Velocity, rpsUnitModel.toTicks(speed) / 10.); //convert rps to ticks per 100ms
    }

}
