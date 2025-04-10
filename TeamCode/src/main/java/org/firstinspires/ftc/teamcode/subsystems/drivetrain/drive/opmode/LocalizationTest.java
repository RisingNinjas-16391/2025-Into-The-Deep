package org.firstinspires.ftc.teamcode.subsystems.drivetrain.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@TeleOp(group = "drive")

public class LocalizationTest extends CommandOpMode {

    private DrivetrainSubsystem drive;
    private MecanumDriveCommand driveCommand;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        drive = new DrivetrainSubsystem(hardwareMap, telemetry, false);

        gamepad = new GamepadEx(gamepad1);

        schedule(new RunCommand(() -> {
                drive.update();
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
        }));

        driveCommand = new MecanumDriveCommand(
                drive, () -> -gamepad.getLeftY(),
                gamepad::getLeftX, gamepad::getRightX
        );

        schedule(driveCommand);
    }

}
