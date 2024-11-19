package org.firstinspires.ftc.teamcode.commands.automation;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoVelocityCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import com.acmerobotics.dashboard.config.Config;

@Config

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakePivotSubsystem intakePivotSubsystem, IntakeSubsystem intakeSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePivotPositionCommand(intakePivotSubsystem, OperatorPresets.Transfer),
                                new WaitCommand(300),
                                new ExtendoPositionCommand(extendoSubsystem, () -> 0),
                                new ExtendoVelocityCommand(extendoSubsystem, () -> -50).withTimeout(500)
                        ),
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevatorSubsystem, () -> 20),
                                new OuttakePivotPositionCommand(outtakePivotSubsystem, 158),
                                new ClawPositionCommand(outtakeClawSubsystem, () -> 90))
                 ).withTimeout(6000),

                new ElevatorPositionCommand(elevatorSubsystem, () -> 0).withTimeout(500),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 160).withTimeout(200),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 156).withTimeout(200),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 65),
                new WaitCommand(400),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 15),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 355));
    }
}
