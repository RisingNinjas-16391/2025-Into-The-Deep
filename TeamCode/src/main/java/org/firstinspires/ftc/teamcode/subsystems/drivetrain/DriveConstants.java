package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.lib.pathplannerlib.path.PathConstraints;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.util.HolonomicPathFollowerConfig;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.util.PIDConstants;
import org.firstinspires.ftc.teamcode.lib.pathplannerlib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public final static HolonomicPathFollowerConfig CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0, 0),
            new PIDConstants(0.5, 0, 0),
            2,
            0.5,
            new ReplanningConfig());

    public final static PathConstraints AUTO_CONSTRAINTS = new PathConstraints(
            1, 1,
            Units.degreesToRadians(180), Units.degreesToRadians(180));
}
