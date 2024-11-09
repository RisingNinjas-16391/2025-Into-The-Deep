package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;

    public MecanumDriveCommand(DrivetrainSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(0.6 * leftY.getAsDouble(), - 0.6 * leftX.getAsDouble(), 0.6 * rightX.getAsDouble());
        drive.updatePoseEstimate();
    }

}
