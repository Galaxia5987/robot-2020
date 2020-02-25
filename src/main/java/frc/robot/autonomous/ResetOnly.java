package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.VisionTurret;
import frc.robot.utilities.VisionModule;

import java.util.Collections;

public class ResetOnly extends SequentialCommandGroup {

    public ResetOnly(Drivetrain drivetrain, Turret turret) {
        addCommands(
                new VisionTurret(turret, true),
                new InstantCommand(() -> VisionModule.setLEDs(true)),
                new WaitCommand(1),
                new InitiatePosition(drivetrain, Collections.emptyList(), 180)
        );
    }

}
