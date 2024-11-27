package org.firstinspires.ftc.teamcode.commands.intake;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {

    private final IntakeSubsystem m_intake;
    private DoubleSupplier m_power;

    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier power){
        m_intake = intake;
        m_power = power;

        addRequirements(m_intake);
    }

    public IntakeCommand(IntakeSubsystem intake, double power){
        m_intake = intake;
        m_power = () -> power;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_intake.setPower(m_power.getAsDouble());
    }

    @Override
    public void end(boolean cancelled) {
        m_intake.setPower(0);
    }
}