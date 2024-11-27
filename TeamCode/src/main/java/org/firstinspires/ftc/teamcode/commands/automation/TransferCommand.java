package org.firstinspires.ftc.teamcode.commands.automation;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import com.acmerobotics.dashboard.config.Config;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@Config

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(IntakePivotSubsystem intakePivotSubsystem, IntakeSubsystem intakeSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePivotPositionCommand(intakePivotSubsystem, OperatorPresets.Transfer),
                                new WaitCommand(0.3),
                                new ExtendoPositionCommand(extendoSubsystem, () -> 0)
                        ),
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevatorSubsystem, () -> 14),
                                new OuttakePivotPositionCommand(outtakePivotSubsystem, 149),
                                new ClawPositionCommand(outtakeClawSubsystem, () -> 90))
                 ).withTimeout(6000),

                new ElevatorPositionCommand(elevatorSubsystem, () -> 0).withTimeout(500),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 45),
                new WaitCommand(400),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 10),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 355));
    }
}
