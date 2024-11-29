package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorPositionCommand extends Command {
    private ElevatorSubsystem m_elevator;
    private DoubleSupplier m_positionSupplier;
    private double m_position;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, DoubleSupplier position) {
        m_elevator = elevator;
        m_positionSupplier = position;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(m_positionSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isFinished();
    }
}
