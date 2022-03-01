package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                                .splineTo(new Vector2d(-12,-40),Math.toRadians(90))
                                .setReversed(true)
                                .splineTo(new Vector2d(-55,-55),Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(56,-60),Math.toRadians(0))
                                .back(50)
                                .lineToLinearHeading(new Pose2d(-10, -45, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(10, -60, Math.toRadians(0)))
                                .forward(35)
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}