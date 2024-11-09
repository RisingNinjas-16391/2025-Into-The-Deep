package org.firstinspires.ftc.teamcode.commands.pivot;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;

public class IntakePivotPositionCommand extends CommandBase {
    private IntakePivotSubsystem m_intakePivot;
    private double m_position;

    public IntakePivotPositionCommand(IntakePivotSubsystem pivot, double position) {
        m_intakePivot = pivot;
        m_position = position;
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
