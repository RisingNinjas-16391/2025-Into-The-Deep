package org.firstinspires.ftc.teamcode.commands.auto;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive) {
        addCommands();
    }
}
