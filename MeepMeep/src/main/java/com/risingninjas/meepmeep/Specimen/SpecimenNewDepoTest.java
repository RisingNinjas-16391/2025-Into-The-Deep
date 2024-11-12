package com.risingninjas.meepmeep.Specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SpecimenNewDepoTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(184.02607784577722), Math.toRadians(184.02607784577722), 15.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, -60, 1.57))
                                //HighPreset
                                .forward(30)
                                //ScoreDownHighHook
                                .waitSeconds(.5)
                                .back(5)
                                //Claw Release
                                .waitSeconds(.5)
                                //IntakeSpecimenPreset
                                .strafeTo(new Vector2d(20, -50))
                                .back(8)
                                //Close Claw
                                .waitSeconds(.5)
                                .strafeTo(new Vector2d(0, -40))
                                .forward(10)
                                //.strafeLeft(60)
                            .build()

                );

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}