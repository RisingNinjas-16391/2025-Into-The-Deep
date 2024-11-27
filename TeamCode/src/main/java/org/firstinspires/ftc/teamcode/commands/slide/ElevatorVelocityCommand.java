package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorVelocityCommand extends Command {
    private ElevatorSubsystem m_elevator;
    private DoubleSupplier m_velocity;

    public ElevatorVelocityCommand(ElevatorSubsystem elevator, DoubleSupplier velocity) {
        m_elevator = elevator;
        m_velocity = velocity;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.incrementPosition(m_velocity.getAsDouble() * 0.02);
    }
}
