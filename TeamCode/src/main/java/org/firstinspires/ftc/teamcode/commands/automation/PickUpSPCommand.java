package org.firstinspires.ftc.teamcode.commands.automation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.commands.claw.ClawPositionCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.OuttakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.wrist.WristPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@Config
public class PickUpSPCommand extends SequentialCommandGroup {
    public PickUpSPCommand( ClawSubsystem outtakeClawSubsystem, ElevatorSubsystem elevatorSubsystem, OuttakePivotSubsystem outtakePivotSubsystem, WristSubsystem wristSubsystem) {
        addCommands(
                new ClawPositionCommand(outtakeClawSubsystem, () -> 45),
                new WaitCommand(0.3),
                new ElevatorPositionCommand(elevatorSubsystem, () -> OperatorPresets.HighBar),
                new OuttakePivotPositionCommand(outtakePivotSubsystem, () -> 285),
                new WristPositionCommand(wristSubsystem, ()-> OperatorPresets.ScoreSpecimen)


        );
    }
}
