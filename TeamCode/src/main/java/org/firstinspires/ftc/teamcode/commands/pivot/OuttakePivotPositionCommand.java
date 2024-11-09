package org.firstinspires.ftc.teamcode.commands.pivot;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;

import java.util.function.DoubleSupplier;

public class OuttakePivotPositionCommand extends CommandBase {
    private OuttakePivotSubsystem m_outtakePivot;
    private DoubleSupplier m_position;

    public OuttakePivotPositionCommand(OuttakePivotSubsystem pivot, double position) {
        m_outtakePivot = pivot;
        m_position = () -> position;
    }

    public OuttakePivotPositionCommand(OuttakePivotSubsystem pivot, DoubleSupplier position) {
        m_outtakePivot = pivot;
        m_position = position;
    }

    @Override
    public void initialize() {
        m_outtakePivot.turnToAngle(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
