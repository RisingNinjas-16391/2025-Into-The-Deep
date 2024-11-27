package org.firstinspires.ftc.teamcode.commands.claw;

import org.firstinspires.ftc.teamcode.subsystems.claws.ClawSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClawPositionCommand extends Command {
    private ClawSubsystem m_claw;
    private DoubleSupplier m_position;

    public ClawPositionCommand(ClawSubsystem claw, DoubleSupplier position) {
        m_claw = claw;
        m_position = position;
    }

    @Override
    public void initialize() {
        m_claw.turnToAngle(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
