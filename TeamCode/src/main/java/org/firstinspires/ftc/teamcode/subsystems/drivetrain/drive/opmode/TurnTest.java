package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.drivetrain.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This is a simple routine to test turning capabilities.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Config
@Autonomous(group = "drive")
@Disabled

public class TurnTest extends CommandOpMode {

    public static double ANGLE = 90; // deg

    private DrivetrainSubsystem drive;
    private TurnCommand turnCommand;

    @Override
    public void initialize() {
        drive = new DrivetrainSubsystem(hardwareMap, telemetry, false);
        turnCommand = new TurnCommand(drive, Math.toRadians(ANGLE));
        schedule(turnCommand.whenFinished(() -> {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }));
    }

}
