package org.firstinspires.ftc.teamcode.commands.wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.wrist.WristSubsystem;

import java.util.function.DoubleSupplier;

public class WristPositionCommand extends CommandBase {
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
}
