package org.firstinspires.ftc.teamcode.commands.automation;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class TopTransferCommand extends SequentialCommandGroup {
    public TopTransferCommand(ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(

                new ElevatorPositionCommand(elevatorSubsystem, () -> 14),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 149),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 90),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 0).withTimeout(500),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 45),
                new WaitCommand(400),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 355)




        );
    }
}
