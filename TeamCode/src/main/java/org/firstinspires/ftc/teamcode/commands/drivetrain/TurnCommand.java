package org.firstinspires.ftc.teamcode.commands.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnCommand extends Command {
    private final DrivetrainSubsystem m_drive;
    private final DoubleSupplier m_heading;

    public TurnCommand(DrivetrainSubsystem drive, DoubleSupplier heading) {
        m_drive = drive;
        m_heading = heading;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
//        m_drive.setHeading(m_heading.getAsDouble());
    }

    @Override
    public boolean isFinished() {
//        return m_drive.headingIsFinished();
        return false;
    }
}
