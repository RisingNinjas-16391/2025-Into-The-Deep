package org.firstinspires.ftc.teamcode.commands.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private DrivetrainSubsystem m_drivetrain;
    private Supplier<ChassisSpeeds> m_moveSupply;
    private BooleanSupplier m_fieldRelative;

    private SlewRateLimiter m_xLimiter;
    private SlewRateLimiter m_yLimiter;
    private SlewRateLimiter m_rotLimiter;

    public DriveCommand(DrivetrainSubsystem drive, Supplier<ChassisSpeeds> speedsSupplier, BooleanSupplier fieldRelativeSupplier) {
        m_drivetrain = drive;
        m_moveSupply = speedsSupplier;
        m_fieldRelative = fieldRelativeSupplier;

        m_xLimiter = new SlewRateLimiter(3.0, -6.0, 0.0);
        m_yLimiter = new SlewRateLimiter(3.0, -6.0, 0.0);
        m_rotLimiter = new SlewRateLimiter(0.5);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        ChassisSpeeds move = m_moveSupply.get();

        move.vxMetersPerSecond = m_xLimiter.calculate(move.vxMetersPerSecond);
        move.vyMetersPerSecond = m_yLimiter.calculate(move.vyMetersPerSecond);
//        move.omegaRadiansPerSecond = m_rotLimiter.calculate(move.omegaRadiansPerSecond);

        m_drivetrain.driveFieldRelative(
                move);
    }

}
