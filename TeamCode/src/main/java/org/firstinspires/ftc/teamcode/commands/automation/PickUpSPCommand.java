package org.firstinspires.ftc.teamcode.commands.automation;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

public class PickUpSPCommand extends SequentialCommandGroup {
    public PickUpSPCommand(IntakePivotSubsystem intakePivotSubsystem, ClawSubsystem intakeClawSubsystem, WristSubsystem wristSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ClawPositionCommand(outtakeClawSubsystem, () -> 65),
                new WaitCommand(300),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, () -> 355),
                new ElevatorPositionCommand(elevatorSubsystem, () -> OperatorPresets.HighBar),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, () -> 20)


        );
    }
}