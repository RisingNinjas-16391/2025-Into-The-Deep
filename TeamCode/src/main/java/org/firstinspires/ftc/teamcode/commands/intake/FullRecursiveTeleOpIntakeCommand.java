package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.commands.automation.TransferCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class FullRecursiveTeleOpIntakeCommand extends SequentialCommandGroup {
    public FullRecursiveTeleOpIntakeCommand(
            IntakePivotSubsystem intakePivotSubsystem,
            IntakeSubsystem intakeSubsystem,
            ClawSubsystem outtakeClawSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ExtendoSubsystem extendoSubsystem,
            OuttakePivotSubsystem outtakePivotSubsystem,
            ColorSubsystem colorSensor,
            WristSubsystem outtakeWristSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(colorSensor::sampleDetected),
                        new IntakeCommand(intakeSubsystem, () -> -1),
                        //,
                        new IntakePivotPositionCommand(intakePivotSubsystem, OperatorPresets.Vertical)
                ),
                new ConditionalCommand(
                        new TransferCommand(
                                outtakeWristSubsystem,
                                intakePivotSubsystem,
                                intakeSubsystem,
                                outtakeClawSubsystem,
                                elevatorSubsystem,
                                extendoSubsystem,
                                outtakePivotSubsystem),
                        new SequentialCommandGroup(
                                new IntakeCommand(intakeSubsystem, () -> 1).withTimeout(0.5),
                                new DeferredCommand(() -> new FullRecursiveIntakeCommand(
                                        intakePivotSubsystem,
                                        intakeSubsystem,
                                        outtakeClawSubsystem,
                                        elevatorSubsystem,
                                        extendoSubsystem,
                                        outtakePivotSubsystem,
                                        colorSensor,
                                        outtakeWristSubsystem
                                ), Set.of())
                        ),
                        colorSensor::hasCorrectColor
                )
        );
    }
}
