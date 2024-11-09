package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorVelocityCommand extends CommandBase {
    private ElevatorSubsystem m_elevator;
    private DoubleSupplier m_velocity;

    public ElevatorVelocityCommand(ElevatorSubsystem elevator, DoubleSupplier velocity) {
        m_elevator = elevator;
        m_velocity = velocity;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.disable();
        m_elevator.setVelocity(m_velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.enable();
        m_elevator.setGoal(m_elevator.getExtensionMeters());
    }
}
