package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.commands.automation.PartialTransferCommand;
import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class PartialRecursiveIntakeCommand extends SequentialCommandGroup {
    public PartialRecursiveIntakeCommand(
            IntakePivotSubsystem intakePivotSubsystem,
            IntakeSubsystem intakeSubsystem,
            ClawSubsystem outtakeClawSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ExtendoSubsystem extendoSubsystem,
            OuttakePivotSubsystem outtakePivotSubsystem,
            ColorSubsystem colorSensor) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(colorSensor::sampleDetected),
                        new IntakeCommand(intakeSubsystem, () -> -1),
                        new IntakePivotPositionCommand(intakePivotSubsystem, OperatorPresets.Feeding),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                    new ExtendoPositionCommand(extendoSubsystem, () -> 20).withTimeout(0.5),
                                    new ExtendoPositionCommand(extendoSubsystem, () -> 40).withTimeout(0.5)))
                ),
                new ConditionalCommand(
                        new PartialTransferCommand(
                                intakePivotSubsystem,
                                intakeSubsystem,
                                outtakeClawSubsystem,
                                elevatorSubsystem,
                                extendoSubsystem,
                                outtakePivotSubsystem),
                        new SequentialCommandGroup(
                                new IntakeCommand(intakeSubsystem, () -> 1).withTimeout(0.5),
                                new DeferredCommand(() -> new PartialRecursiveIntakeCommand(
                                        intakePivotSubsystem,
                                        intakeSubsystem,
                                        outtakeClawSubsystem,
                                        elevatorSubsystem,
                                        extendoSubsystem,
                                        outtakePivotSubsystem,
                                        colorSensor
                                ), Set.of())
                        ),
                        colorSensor::hasCorrectColor
                )
        );
    }
}
