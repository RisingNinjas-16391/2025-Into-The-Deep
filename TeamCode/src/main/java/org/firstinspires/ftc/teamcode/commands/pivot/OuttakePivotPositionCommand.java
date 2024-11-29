package org.firstinspires.ftc.teamcode.commands.pivot;

import org.firstinspires.ftc.teamcode.subsystems.pivot.OuttakePivotSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class OuttakePivotPositionCommand extends Command {
    private OuttakePivotSubsystem m_outtakePivot;
    private DoubleSupplier m_position;

    public OuttakePivotPositionCommand(OuttakePivotSubsystem pivot, double position) {
        m_outtakePivot = pivot;
        m_position = () -> position;

        addRequirements(m_outtakePivot);
    }

    public OuttakePivotPositionCommand(OuttakePivotSubsystem pivot, DoubleSupplier position) {
        m_outtakePivot = pivot;
        m_position = position;

        addRequirements(m_outtakePivot);
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
