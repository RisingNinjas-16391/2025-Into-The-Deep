package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class TeleOpHeadingCommand extends CommandBase {
    private final DrivetrainSubsystem drive;

    private final DoubleSupplier angle;

    public TeleOpHeadingCommand(final DrivetrainSubsystem drive, final DoubleSupplier angle) {
        this.drive = drive;

        this.angle = angle;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setHeading(drive.getHeading());
    }
    @Override
    public void execute() {
        drive.setHeading(Math.toRadians(angle.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return drive.isFinishedHeadingPID();
    }
}
