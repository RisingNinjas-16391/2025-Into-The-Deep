package org.firstinspires.ftc.teamcode.subsystems.slides.extendo;

import com.acmerobotics.dashboard.config.Config;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

@Config
public final class ExtendoConstants {
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(5000, 5000);
    public static double kP = 2;
    public static double kI = 0.03;
    public static double kD = 0;
    public static double kF = 0;
    public static double kS = 0;
    public static double kV = 0.05;
    public static double kA = 0;
    public static double kG = 0;

    public static final double ERROR_MARGIN = 2.0;
    public static final double LOWER_BOUND = 0.0;
    public static final double UPPER_BOUND = 55.0;

    public static final double CENTIMETERS_PER_TICK = 62.0 / 946.0;

}
