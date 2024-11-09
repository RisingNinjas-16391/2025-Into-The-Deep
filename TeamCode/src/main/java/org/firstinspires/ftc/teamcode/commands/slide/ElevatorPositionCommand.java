package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.slides.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorPositionCommand extends CommandBase {
    private ElevatorSubsystem m_elevator;
    private DoubleSupplier m_positionSupplier;
    private double m_position;

    public ElevatorPositionCommand(ElevatorSubsystem elevator, DoubleSupplier position) {
        m_elevator = elevator;
        m_positionSupplier = position;
    }

    @Override
    public void initialize() {
        m_elevator.setGoal(m_positionSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_elevator.isAtPosition();
    }
}
