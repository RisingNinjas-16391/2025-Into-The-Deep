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
        addCommands(new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 160),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 90),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 1),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 65),
                new WaitCommand(900),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 355));
    }
}
