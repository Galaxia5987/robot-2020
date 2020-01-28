package frc.robot.commandGroups;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.color_wheel.ColorWheel;
import frc.robot.subsystems.color_wheel.commands.RotationControl;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.DriveStraightCommand;

public class DriveAndRotate extends ParallelDeadlineGroup {


    public DriveAndRotate(Drivetrain drivetrain, ColorWheel colorWheel) {
        super(new RotationControl(colorWheel), new DriveStraightCommand(drivetrain, 0.2,0.2));

    }
}
