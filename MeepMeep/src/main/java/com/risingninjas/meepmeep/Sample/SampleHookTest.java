package com.risingninjas.meepmeep.Sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SampleHookTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 15.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(-90)))
                                // Add movements here
                                .back(30)
                                .forward(10)
                                //ScoreSpecimen
                                .splineTo(new Vector2d(-30, -45), Math.toRadians(180))
                                .back(1)
                                .turn(3.14)
                                .back(1)
                                .splineTo(new Vector2d(-50, -55), Math.toRadians(225))







                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}