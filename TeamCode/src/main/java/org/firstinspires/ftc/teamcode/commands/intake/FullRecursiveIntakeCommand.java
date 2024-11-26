package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.helpers.DeferredCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.Set;

public class FullRecursiveIntakeCommand extends SequentialCommandGroup {
    public FullRecursiveIntakeCommand(IntakePivotSubsystem intakePivotSubsystem, IntakeSubsystem intakeSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem, ColorSubsystem colorSensor) {
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
