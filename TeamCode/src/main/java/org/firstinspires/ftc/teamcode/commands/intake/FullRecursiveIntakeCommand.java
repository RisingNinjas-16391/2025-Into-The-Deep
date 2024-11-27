package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FullRecursiveIntakeCommand extends SequentialCommandGroup {
    public FullRecursiveIntakeCommand(
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
                        new IntakeCommand(intakeSubsystem, () -> -1)
                ),
                new ConditionalCommand(
                        new TransferCommand(
                                intakePivotSubsystem,
                                intakeSubsystem,
                                outtakeClawSubsystem,
                                elevatorSubsystem,
                                extendoSubsystem,
                                outtakePivotSubsystem),
                        new SequentialCommandGroup(
                                new IntakeCommand(intakeSubsystem, () -> 1).withTimeout(500),
                                new DeferredCommand(() -> new FullRecursiveIntakeCommand(
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
