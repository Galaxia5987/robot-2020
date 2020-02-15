package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityFunctions;
import frc.robot.subsystems.UnitModel;
import frc.robot.utilities.VictorConfiguration;
import frc.robot.valuetuner.WebConstantPIDTalon;
import org.techfire225.webapp.FireLog;

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
    private double targetVelocity; // Allows commands to know what the target velocity of the talon is.

    public Shooter() {
        shooterMaster.configFactoryDefault();
        shooterSlave1.configFactoryDefault();
        shooterSlave2.configFactoryDefault();

        VictorConfiguration slaveConfigs = new VictorConfiguration();

        shooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT);
        shooterMaster.setInverted(IS_MASTER_INVERTED);
        shooterMaster.setSensorPhase(IS_ENCODER_INVERTED);

        // Closed loop control
        shooterMaster.config_kP(TALON_PID_SLOT, KP, TALON_TIMEOUT);
        shooterMaster.config_kI(TALON_PID_SLOT, KI, TALON_TIMEOUT);
        shooterMaster.config_kD(TALON_PID_SLOT, KD, TALON_TIMEOUT);
        shooterMaster.config_kF(TALON_PID_SLOT, KF, TALON_TIMEOUT);

        shooterMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        shooterMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

        // Slave configuration
        shooterSlave1.follow(shooterMaster);
        shooterSlave2.follow(shooterMaster);
        shooterSlave1.setInverted(IS_SLAVE_1_INVERTED);
        shooterSlave2.setInverted(IS_SLAVE_2_INVERTED);

        UtilityFunctions.configAllVictors(slaveConfigs, shooterSlave1, shooterSlave2);
        new WebConstantPIDTalon("shooterTalon", KP, KI, KD, KF, shooterMaster);
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
        targetVelocity = speed;
        shooterMaster.set(ControlMode.Velocity, rpsUnitModel.toTicks100ms(targetVelocity));
    }

    /**
     * @param distance the distance away from the target.
     * @return the calculated velocity to get to the target in rps.
     */
    public double approximateVelocity(double distance) {
        return 0.1069*Math.pow(distance,4) - 2.4364*Math.pow(distance,3) + 19.814*Math.pow(distance,2) - 63.171*distance + 131.49; //The current reading as of 15.2.20
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

    public boolean isShooterReady(){
        return Math.abs(getSpeed() - getTargetVelocity()) <= VELOCITY_TOLERANCE;
    }

    public void setPower(double power) {
        shooterMaster.set(ControlMode.PercentOutput, power);
    }

    public void stop() {
        setPower(0);
    }

    public double getVisionDistance(){
        return visionDistance.getDouble(0);
    }

    public double getMasterVoltage() {
        return shooterMaster.getMotorOutputVoltage();
    }
}
