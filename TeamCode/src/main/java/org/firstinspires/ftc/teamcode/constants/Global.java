package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Global {

    public enum ScoringType {
        HIGHBAR,
        LOWBAR,
        HIGHBUCKET,
        LOWBUCKET

    }

    public static ScoringType ScoringState = ScoringType.HIGHBAR;

    public static double HighBar = 27;
    public static double HighBarAuto = 28;
    public static double LowBar = 9.8;
    public static double HighBucket = 11;
    public static double LowBucket = 40;

    public static double IntakeSpecimen = 300;

    public static double ScoreSpecimen = 210;
}
