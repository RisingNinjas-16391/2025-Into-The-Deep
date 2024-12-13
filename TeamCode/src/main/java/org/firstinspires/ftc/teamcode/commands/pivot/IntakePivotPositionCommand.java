package org.firstinspires.ftc.teamcode.commands.pivot;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivotPositionCommand extends Command {
    private IntakePivotSubsystem m_intakePivot;
    private double m_position;

    public IntakePivotPositionCommand(IntakePivotSubsystem pivot, double position) {
        m_intakePivot = pivot;
        m_position = position;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_intakePivot.turnToAngle(m_position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
