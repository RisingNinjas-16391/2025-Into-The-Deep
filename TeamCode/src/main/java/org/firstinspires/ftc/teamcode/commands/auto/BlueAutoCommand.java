package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

<<<<<<< Updated upstream
import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectorySequenceCommand;
=======
import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.drivetrain.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.constants.Global;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

public class BlueAutoCommand extends SequentialCommandGroup {
    public BlueAutoCommand(DrivetrainSubsystem drive) {
        addCommands(
<<<<<<< Updated upstream
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .forward(10)
=======
                new ElevatorPositionCommand(elevator, () -> Global.HighBar),
                new FollowTrajectorySequenceCommand(drive, drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Add movements here
                        .back(30)
                        .build()
                ),
                new OuttakePivotPositionCommand(outtakePivot, () -> Global.ScoreSpecimen),
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
>>>>>>> Stashed changes
                        .build()
                )
        );
    }
}