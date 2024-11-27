package org.firstinspires.ftc.teamcode.commands.slide;

import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendoVelocityCommand extends Command {
    private ExtendoSubsystem m_extendo;
    private DoubleSupplier m_velocity;

    public ExtendoVelocityCommand(ExtendoSubsystem extendo, DoubleSupplier velocity) {
        m_extendo = extendo;
        m_velocity = velocity;

        addRequirements(extendo);
    }

    @Override
    public void initialize() {
        m_extendo.incrementPosition(m_velocity.getAsDouble() * 0.02);
    }
}
