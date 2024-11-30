package org.firstinspires.ftc.teamcode.commands.automation;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.pivot.IntakePivotPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ExtendoPositionCommand;
import org.firstinspires.ftc.teamcode.constants.OperatorPresets;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OuttakePieceCommand extends SequentialCommandGroup {
    public OuttakePieceCommand(ExtendoSubsystem m_extendo, IntakePivotSubsystem m_intakePivot, IntakeSubsystem m_intake) {
        addCommands(
                new ParallelCommandGroup(
                        new ExtendoPositionCommand(m_extendo, () -> 15),
                        new IntakePivotPositionCommand(m_intakePivot, OperatorPresets.Feeding)
                ),
                new IntakeCommand(m_intake, () -> 1).withTimeout(0.25)
        );
    }
}
