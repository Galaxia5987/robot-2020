package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;
import static frc.robot.RobotContainer.*;


public class JoystickTurret extends CommandBase {
    private RobotContainer m_robotContainer = new RobotContainer();
    private static Turret turret;

    public JoystickTurret(Turret turret) {
        JoystickTurret.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double joystickInput = getLeftXboxY();
        double position = turret.getAngle() + joystickInput * TURRET_JOYSTICK_SPEED;
        if (position < MINIMUM_POSITION && position > MAXIMUM_POSITION)
            turret.setAngle(position);
    }

}
