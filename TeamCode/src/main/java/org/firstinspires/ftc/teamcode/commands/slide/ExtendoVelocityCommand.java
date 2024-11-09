package org.firstinspires.ftc.teamcode.commands.slide;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.slides.extendo.ExtendoSubsystem;

import java.util.function.DoubleSupplier;

public class ExtendoVelocityCommand extends CommandBase {
    private ExtendoSubsystem m_extendo;
    private DoubleSupplier m_velocity;

    public ExtendoVelocityCommand(ExtendoSubsystem extendo, DoubleSupplier velocity) {
        m_extendo = extendo;
        m_velocity = velocity;

        addRequirements(extendo);
    }

    @Override
    public void initialize() {
        m_extendo.disable();
        m_extendo.setVelocity(m_velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_extendo.enable();
        m_extendo.setGoal(m_extendo.getExtensionMeters());
    }
}
