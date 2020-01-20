package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.ANGLE_THRESHOLD;

public class Wait extends CommandBase {
    private Shooter shooter;
    private Turret turret;
    private double distance;
    private double angle;
    public static final NetworkTable waitTable = NetworkTableInstance.getDefault().getTable("shooter");
    private static final NetworkTableEntry visionAngle = waitTable.getEntry("angle");

    public Wait(Shooter shooter, Turret turret, double distance) {
        addRequirements(shooter, turret);
        this.shooter = shooter;
        this.turret = turret;
        this.distance = distance;
        this.angle = visionAngle.getDouble(3);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return shooter.getSpeed() >= shooter.approximateVelocity(distance) && Math.abs(turret.getEncoderPosition() - angle) <= ANGLE_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
