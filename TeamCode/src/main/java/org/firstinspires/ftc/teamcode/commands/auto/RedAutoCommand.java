package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;

public class RedAutoCommand extends SequentialCommandGroup {
    public RedAutoCommand(DrivetrainSubsystem drive, ElevatorSubsystem elevator, OuttakePivotSubsystem outtakePivot, ClawSubsystem outtakeClaw) {
        addCommands(
                new ElevatorPositionCommand(elevator, () -> OperatorPresets.HighBar),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        // Add movements here
                        .back(30)
                        .build()
                ),
                new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.ScoreSpecimen),
                new WaitCommand(700),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-30, 0, Math.toRadians(0)))
                        // Add movements here
                        .forward(5)
                        .build()
                ),
                new ClawPositionCommand(outtakeClaw, () -> 90),

                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-25, 0, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-15, 40, Math.toRadians(180)))
                        .build()),
                new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.IntakeSpecimen),
                new ClawPositionCommand(outtakeClaw, () -> 90).withTimeout(400),
                new ElevatorPositionCommand(elevator, () -> 0),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-15, 40, Math.toRadians(180)))
                        // Add movements here
                        .back(5)
                        .build()
                ),
                new ClawPositionCommand(outtakeClaw, () -> 65).withTimeout(400),
                new ElevatorPositionCommand(elevator, () -> OperatorPresets.HighBar),
                new OuttakePivotPositionCommand(outtakePivot, () -> 355),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-10, 40, Math.toRadians(180)))
                // Add movements here
                .lineToSplineHeading(new Pose2d(-20, -5, Math.toRadians(0)))
                        .back(10)
                .build()),

        new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.ScoreSpecimen),
                new WaitCommand(700),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-30, 0, Math.toRadians(0)))
                        // Add movements here
                        .forward(5)
                        .build()
                ),
                new ClawPositionCommand(outtakeClaw, () -> 90),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-25, 0, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-5, 40, Math.toRadians(0)))
                .build())


                /*

                new ClawPositionCommand(outtakeClaw, () -> 90),
                new WaitCommand(500),
                new ElevatorPositionCommand(elevator, () -> 0),
                        .lineToSplineHeading(new Pose2d(20, -50, Math.toRadians(90)))
                        .back(8)
                        //Close Claw
                        .waitSeconds(.5)
                        .lineToSplineHeading(new Pose2d(0, -40, Math.toRadians(-90)))
                        .back(10)
                        .forward(5)
                        .splineTo(new Vector2d(45, -55), Math.toRadians(0))
                        .build()*/

        );
    }
}