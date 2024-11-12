package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class SpecimenPreLoadandPark extends SequentialCommandGroup {
    public SpecimenPreLoadandPark(DrivetrainSubsystem drive, ElevatorSubsystem elevator, OuttakePivotSubsystem outtakePivot, ClawSubsystem outtakeClaw) {
        addCommands(
                new ElevatorPositionCommand(elevator, () -> OperatorPresets.HighBar),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .build()
                ),
                new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.ScoreSpecimen),
                new WaitCommand(700),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-30, 0, 0))
                        // Add movements here
                        .forward(5)
                        .build()
                ),
                new ClawPositionCommand(outtakeClaw, () -> 90),
                new WaitCommand(500),
                new ElevatorPositionCommand(elevator, () -> 0),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(-30, 0, 0))
                        // Add movements here
                        .forward(25)
                        .strafeLeft(60)
                        .build()
                )
        );
    }
}