package org.firstinspires.ftc.teamcode.commands.automation;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class PickUpSPCommand extends SequentialCommandGroup {
    public PickUpSPCommand( ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, OuttakePivotSubsystem outtakePivotSubsystem) {
        addCommands(
                new ClawPositionCommand(outtakeClawSubsystem, () -> 65),
                new WaitCommand(300),
                new ElevatorPositionCommand(elevatorSubsystem, () -> OperatorPresets.HighBarBack),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, () -> 15)


        );
    }
}
