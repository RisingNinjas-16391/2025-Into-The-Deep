package org.firstinspires.ftc.teamcode.commands.climb;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.pivot.IntakePivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbPowerCommand extends Command {
    private ClimbSubsystem m_climb;
    private DoubleSupplier m_power;

    public ClimbPowerCommand(ClimbSubsystem climb, DoubleSupplier power) {
        m_climb = climb;
        m_power = power;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        m_climb.setPower(m_power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}