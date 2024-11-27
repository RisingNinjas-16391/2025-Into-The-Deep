package org.firstinspires.ftc.teamcode.commands.automation;

import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropDropCommand extends SequentialCommandGroup {
    public DropDropCommand(IntakePivotSubsystem intakePivotSubsystem, IntakeSubsystem intakeSubsystem, ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, ExtendoSubsystem extendoSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(

                new IntakePivotPositionCommand(intakePivotSubsystem, 15),
                new WaitCommand(0.5),
                new WaitCommand(0.5),
                new IntakePivotPositionCommand(intakePivotSubsystem, 27));
    }
}
