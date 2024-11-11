package org.firstinspires.ftc.teamcode.commands.automation;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakePivotSubsystem intakePivotSubsystem, ClawSubsystem intakeClawSubsystem, WristSubsystem wristSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePivotPositionCommand(intakePivotSubsystem, 0),
                                new WaitCommand(300),
                                new ClawPositionCommand(intakeClawSubsystem, () -> 180),
                                new WristPositionCommand(wristSubsystem, () -> 150),
                                new WaitCommand(300),
                                new IntakePivotPositionCommand(intakePivotSubsystem, 27),
                                new ExtendoPositionCommand(extendoSubsystem, () -> 0)
                        ),
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                                new OuttakePivotPositionCommand(outtakePivotSubsystem, 160),
                                new ClawPositionCommand(outtakeClawSubsystem, () -> 90))
                 ),
                new WaitCommand(300),
                new ClawPositionCommand(intakeClawSubsystem, () -> 90),
                new WaitCommand(300),
                new IntakePivotPositionCommand(intakePivotSubsystem, 20),
                new WaitCommand(300),

                new ElevatorPositionCommand(elevatorSubsystem, () -> 1),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 65),
                new WaitCommand(600),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 355));
    }
}
