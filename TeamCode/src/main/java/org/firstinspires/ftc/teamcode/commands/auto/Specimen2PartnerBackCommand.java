package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.automation.PickUpSPCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;

public class Specimen2PartnerBackCommand extends SequentialCommandGroup {
    public Specimen2PartnerBackCommand(DrivetrainSubsystem drive, ElevatorSubsystem elevator, OuttakePivotSubsystem outtakePivot, ClawSubsystem outtakeClaw) {
        addCommands(

                //Drives to Score while getting Elevator and DepoPivotReady
                new ParallelCommandGroup(
                        new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                // Add movements here
                                .forward(30)
                                .build()
                        ),
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevator, () -> OperatorPresets.HighBarBack).withTimeout(200),
                                new OuttakePivotPositionCommand(outtakePivot, 20)
                        )

                ),
                //Tries to score Piece with wrist
                new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.ScoreSpecimenBack),
                new WaitCommand(300),


                //Drives away from bar while opening Claw to score
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(30, 0, 0))
                        // Add movements here
                        .back(5)
                        .build()
                ),
                new ClawPositionCommand(outtakeClaw, () -> 90),
                new WaitCommand(500),


                //Drives to Pickup Piece 2 while
                //Close claw, Rotate pivot, Put elevator down, Open Claw
                new ParallelCommandGroup(
                        new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(25, 0, 0))
                                // Add movements here
                                .lineToSplineHeading(new Pose2d(5, -40, Math.toRadians(0)))
                                .build()
                        ),
                        new SequentialCommandGroup(
                                new ClawPositionCommand(outtakeClaw, () -> 65),
                                new OuttakePivotPositionCommand(outtakePivot, () -> OperatorPresets.IntakeSpecimen).withTimeout(800),
                                new ElevatorPositionCommand(elevator, () -> 0),
                                new ClawPositionCommand(outtakeClaw, () -> 90)

                        )
                ),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(5, -40, 0))
                        // Add movements here
                        .back(5)
                        .build()
                ),
                //Drives to score while getting elevator and everything in POS
                new ParallelCommandGroup(
                        new PickUpSPCommand(
                                outtakeClaw,
                                elevator,
                                outtakePivot),
                        new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, -40, 0))
                             // Add movements here
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(25, 0))
                                .forward(5)
                                .build()
                                )
                )


        );
    }
}