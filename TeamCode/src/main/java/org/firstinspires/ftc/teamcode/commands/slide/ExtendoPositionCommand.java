package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

public class ExtendoPositionCommand extends CommandBase {
    private ExtendoSubsystem m_extendo;
    private DoubleSupplier m_positionSupplier;
    private double m_position;

    public ExtendoPositionCommand(ExtendoSubsystem extendo, DoubleSupplier position) {
        m_extendo = extendo;
        m_positionSupplier = position;
    }

    @Override
    public void initialize() {
        m_extendo.setGoal(m_positionSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_extendo.isAtPosition();
    }
}
