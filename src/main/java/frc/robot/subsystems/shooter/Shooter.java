package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.VictorConfiguration;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.TALON_TIMEOUT;
import static frc.robot.Ports.Shooter.*;
import static frc.robot.Ports.TALON_PID_SLOT;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterMaster = new TalonSRX(MASTER);
    private final VictorSPX shooterSlave1 = new VictorSPX(SLAVE_1);
    private final VictorSPX shooterSlave2 = new VictorSPX(SLAVE_2);
    private final UnitModel rpsUnitModel = new UnitModel(TICKS_PER_ROTATION);//TODO: correct all velocity usages to use the not yet commited velocity unit model convertion
    private static final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("shooter");
    private static final NetworkTableEntry visionDistance = visionTable.getEntry("distance");
    private final VictorConfiguration slaveConfigs = new VictorConfiguration();

    public Shooter() {
        // Basic motor configurations
        // Master configurations
        shooterMaster.configFactoryDefault();
        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        shooterMaster.setInverted(IS_MASTER_INVERTED);
        shooterMaster.setSensorPhase(IS_ENCODER_INVERTED);

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
        shooterSlave1.follow(shooterMaster);
        shooterSlave2.follow(shooterMaster);
        shooterSlave1.setInverted(IS_SLAVE_1_INVERTED);
        shooterSlave2.setInverted(IS_SLAVE_2_INVERTED);

        shooterMaster.setNeutralMode(NeutralMode.Coast);

        slaveConfigs.setNeutralMode(NeutralMode.Coast);
        slaveConfigs.setEnableVoltageCompensation(true);
        slaveConfigs.setVoltageCompensationSaturation(12);
        UtilityFunctions.configAllVictors(slaveConfigs, shooterSlave1, shooterSlave2);
    }

    /**
     * @return the speed of the shooter in rpm.
     */
    public double getSpeed() {
        return rpsUnitModel.toVelocity(shooterMaster.getSelectedSensorVelocity());
    }

    /**
     * set the speed of the shooter.
     * @param speed the rotations per second of the shooter.
     */
    public void setSpeed(double speed) {
        shooterMaster.set(ControlMode.Velocity, rpsUnitModel.toTicks100ms(speed));
    }

    /**
     * @param distance the distance away from the target.
     * @return the calculated velocity to get to the target in rps.
     */
    public double approximateVelocity(double distance) {
        return (8.68 * Math.exp(0.1685 * distance));
    }

    public double getVisionDistance(){
        return visionDistance.getDouble(0);
    }
}
