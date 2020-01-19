package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.conveyor;
import static frc.robot.Constants.Conveyor.*;

public class MinimizeConveyor extends CommandBase {
    private Timer timer = new Timer();
    private double timeout;
    public MinimizeConveyor(double timeout) {
        addRequirements(conveyor);
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        conveyor.setConveyorSpeed(CONVEYOR_MOTOR_RETURN_VELOCITY);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return conveyor.feederSensedObject() || timer.get() >= timeout || timeout == 0;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        timer.stop();
    }

}