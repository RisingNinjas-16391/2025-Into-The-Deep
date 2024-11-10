package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;
    private final DoubleSupplier scale;

    public MecanumDriveCommand(DrivetrainSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.scale = () -> 0.8;

        addRequirements(drive);
    }

    public MecanumDriveCommand(DrivetrainSubsystem drive, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier rightX, DoubleSupplier scale) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.scale = scale;

        addRequirements(drive);
    }

    @Override
    public void execute() {

        drive.drive(scale.getAsDouble() * leftY.getAsDouble(), - scale.getAsDouble() * leftX.getAsDouble(), scale.getAsDouble() * rightX.getAsDouble());
        drive.updatePoseEstimate();
    }

}
