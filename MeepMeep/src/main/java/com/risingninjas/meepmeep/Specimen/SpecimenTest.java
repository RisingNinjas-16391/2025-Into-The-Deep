package com.risingninjas.meepmeep.Specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 15.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(-90)))
                                //HighPreset
                                .back(30)
                                //ScoreDownHighHook
                                .waitSeconds(.5)
                                .forward(5)
                                //Claw Release
                                .waitSeconds(.5)
                                //IntakeSpecimenPreset
                                .lineToSplineHeading(new Pose2d(20, -50, Math.toRadians(90)))
                                .back(8)
                                //Close Claw
                                .waitSeconds(.5)
                                .lineToSplineHeading(new Pose2d(0, -40, Math.toRadians(-90)))
                                .back(10)
                                .forward(5)
                                .splineTo(new Vector2d(45, -55), Math.toRadians(0))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}