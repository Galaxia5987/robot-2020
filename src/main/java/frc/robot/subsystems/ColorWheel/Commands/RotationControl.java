package frc.robot.subsystems.ColorWheel.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.colorWheel;

public class RotationControl extends CommandBase {
    private String[] colors = {"Yellow","Red","Green","Blue"};
    private String initialColor;
    private int initialIndex = 0;
    private double spinCounter = 0.125;

    @Override
    public void initialize() {
        initialColor = colorWheel.getColorString();
        for (int i = 0;i< colors.length;i++){
            if (colors[i].equals(initialColor)) {
                initialIndex = i;
                break;
            }
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
