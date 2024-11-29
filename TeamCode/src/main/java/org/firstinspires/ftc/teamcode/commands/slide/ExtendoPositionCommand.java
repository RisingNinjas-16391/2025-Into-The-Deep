package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendoPositionCommand extends Command {
    private ExtendoSubsystem m_extendo;
    private DoubleSupplier m_positionSupplier;
    private double m_position;

    public ExtendoPositionCommand(ExtendoSubsystem extendo, DoubleSupplier position) {
        m_extendo = extendo;
        m_positionSupplier = position;

//        addRequirements(extendo);
    }

    @Override
    public void initialize() {
        m_extendo.setPosition(m_positionSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_extendo.isFinished();
    }
}
