package org.firstinspires.ftc.teamcode.commands.auto;

import org.firstinspires.ftc.teamcode.lib.wpilib_command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive) {
        addCommands();
    }
}
