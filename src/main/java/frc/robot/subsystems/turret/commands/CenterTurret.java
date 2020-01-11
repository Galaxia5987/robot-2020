package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.turret;

public class CenterTurret extends CommandBase {

    public CenterTurret() {
        addRequirements(turret);
    }

}
