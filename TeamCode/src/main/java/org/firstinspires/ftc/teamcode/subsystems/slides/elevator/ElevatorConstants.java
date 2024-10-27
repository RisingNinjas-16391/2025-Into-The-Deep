package org.firstinspires.ftc.teamcode.subsystems.slides.elevator;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.helpers.BetterElevatorFeedforward;

@Config
public final class ElevatorConstants {
    public static final TrapezoidProfile.Constraints TRAPEZOIDAL_CONSTRAINTS = new TrapezoidProfile.Constraints(5000, 5000);
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0.3;
    public static double kF = 0;
    public static double kS = 0.038;
    public static double kV = 0.4;
    public static double kA = 0.1;
    public static double kG = 1.2;

    public static PIDFController PIDF_CONTROLLER = new PIDFController(1,0,0.001, 0);
    public static BetterElevatorFeedforward FEEDFORWARD_CONTROLLER = new BetterElevatorFeedforward(0.038, 1.3, 1.821, 0.02);

    public static final double ERROR_MARGIN = 2.0;
    public static final double LOWER_BOUND = 0.0;
    public static final double UPPER_BOUND = 70.0;

    public static final double CENTIMETERS_PER_TICK = 73.5 / 3265.0;

}
