package org.firstinspires.ftc.teamcode.commands.wrist;

import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class WristPositionCommand extends Command {
    private WristSubsystem m_wrist;
    private DoubleSupplier m_position;

    public WristPositionCommand(WristSubsystem wrist, DoubleSupplier position) {
        m_wrist = wrist;
        m_position = position;
    }

    @Override
    public void execute() {
        m_wrist.turnToAngle(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
