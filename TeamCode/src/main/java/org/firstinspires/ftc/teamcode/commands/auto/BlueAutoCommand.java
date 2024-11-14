package org.firstinspires.ftc.teamcode.commands.auto;

import org.firstinspires.ftc.teamcode.lib.wpilib_command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive) {
        addCommands();
    }
}
