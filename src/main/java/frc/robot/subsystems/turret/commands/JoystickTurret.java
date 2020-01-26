package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.Turret.*;
import static frc.robot.RobotContainer.*;


public class JoystickTurret extends CommandBase {
    private RobotContainer m_robotContainer = new RobotContainer();
    private final Turret turret;

    public JoystickTurret(Turret turret) { //This does not power the motors, it simply adjusts the target angle. Henceforth, there is no reason to interrupt the turning class.
        this.turret = turret;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double joystickInput = m_robotContainer.getXboxY();
        double delta = joystickInput * TURRET_JOYSTICK_SPEED;
        turret.setDelta(delta);
    }

}
