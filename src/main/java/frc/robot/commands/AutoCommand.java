package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSystem;

public class AutoCommand extends SequentialCommandGroup {
    public AutoCommand(DriveSystem drive) {
        addCommands(
            // Drive forward the specified distance
            new AimToTarget(drive,1),
            new TurnToAngle(135, drive),
            new AimToTarget(drive,1.5),
            new AimToBall(drive),
            new TurnToAngle(0,drive),
            new AimToTarget(drive)
        );
    }
}
