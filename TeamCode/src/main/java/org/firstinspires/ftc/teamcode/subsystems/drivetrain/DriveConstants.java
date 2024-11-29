package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;

@Config
public class DriveConstants {
    public static double kP = 2.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.0;
    public static double kV = 2.0;
    public static double kA = 0.0;

    public static double HEADING_P = 0.1;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.0;


    public static double GEAR_RATIO = 1.0 / 1152.006;
//    1.0 / 1152.006
//    1.0 / 2351.033
//1.12888
    public static double LINEAR_SCALAR = 1.127;
    public static double ANGULAR_SCALAR = 1.0;

    public final static HolonomicPathFollowerConfig CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0),
            new PIDConstants(5, 0, 0),
            2,
            0.3,
            new ReplanningConfig());

    public final static PathConstraints AUTO_CONSTRAINTS = new PathConstraints(
            1, 1,
            Units.degreesToRadians(180), Units.degreesToRadians(180));
}
