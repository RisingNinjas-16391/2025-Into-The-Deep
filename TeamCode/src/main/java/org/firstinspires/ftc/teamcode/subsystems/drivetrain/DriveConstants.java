package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;

import edu.wpi.first.math.geometry.Translation2d;

@Config
public class DriveConstants {
    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.5;
    public static double kV = 2.0;
    public static double kA = 0.0;

    public static double HEADING_P = 0.1;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.0;

    public static double TRANSLATION_P = 5.0;
    public static double TRANSLATION_I = 0.0;
    public static double TRANSLATION_D = 0.0;

    public static double AUTO_HEADING_P = 5.0;
    public static double AUTO_HEADING_I = 0.0;
    public static double AUTO_HEADING_D = 0.0;

    public static double GEAR_RATIO = 1.0 / 1152.006;

    public static double LINEAR_SCALAR = 1.127;
    public static double ANGULAR_SCALAR = 1.0;

    public static double ROBOT_LENGTH = 0.195;
    public static double ROBOT_WIDTH = 0.240;

    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0);
    public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0);
    public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0);
}
