package org.firstinspires.ftc.teamcode.commands.automation;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import com.acmerobotics.dashboard.config.Config;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@Config

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(WristSubsystem outtakeWristSubsystem, IntakePivotSubsystem intakePivotSubsystem, IntakeSubsystem intakeSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePivotPositionCommand(intakePivotSubsystem, OperatorPresets.Transfer),
                                new WaitCommand(0.1),
                                new ExtendoPositionCommand(extendoSubsystem, () -> -2).withTimeout(0.5),
                                new IntakeCommand(intakeSubsystem, () -> -0.5, false).withTimeout(0.2)
                        ),
                         new SequentialCommandGroup(
                                 new ClawPositionCommand(outtakeClawSubsystem, () -> 90),
                                new OuttakePivotPositionCommand(outtakePivotSubsystem, 230),
                                new ElevatorPositionCommand(elevatorSubsystem, () -> 5).withTimeout(0.3),
                                new WristPositionCommand(outtakeWristSubsystem, () -> 80)
                         )
                 ).withTimeout(6),

                new ElevatorPositionCommand(elevatorSubsystem, () -> 0).withTimeout(0.1),
                new ClawPositionCommand(outtakeClawSubsystem, () -> 35),
                new WaitCommand(0.35),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, 0),
                new ElevatorPositionCommand(elevatorSubsystem, () -> 20).withTimeout(0.5)
                );
    }
}
