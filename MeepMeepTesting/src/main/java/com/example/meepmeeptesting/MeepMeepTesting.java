package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        final double width = 12.3;
        final double length = 15.75;
        final double specimenSampleY = -12;
        final Vector2d[] specimenSamplePositions = {new Vector2d(45, specimenSampleY), new Vector2d(55, specimenSampleY), new Vector2d(61, specimenSampleY)};
        final Vector2d specimenHangPos = new Vector2d(8, -34);
        final Vector2d specimenPickupPos = new Vector2d(36, -56);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), width)
                .setDimensions(width, length)
                .build();

        TrajectoryActionBuilder tab1 = myBot.getDrive().actionBuilder(new Pose2d(width / 2, -72 + length / 2, Math.toRadians(90)))
                .lineToY(-38.5)
                .waitSeconds(2)
                .setReversed(true)
                .setTangent(0)
                .lineToXLinearHeading(34, Math.toRadians(10))
                .setTangent(Math.PI / 2)
                .lineToY(-10)
                .setTangent(0)
                .lineToX(40)
                .setTangent(Math.PI / 2)
                .lineToY(-54)
                .lineToY(-10)
                .setTangent(0)
                .lineToX(50)
                .setTangent(Math.PI / 2)
                .lineToY(-60)
                .lineToY(-10);

        /*        .setReversed(true)
                .splineToLinearHeading(new Pose2d(34, -30, 0), Math.PI / 2)
                .setTangent(Math.toRadians(90))
                .lineToY(-12)
                .splineToConstantHeading(new Vector2d(specimenSamplePositions[0].x, -48), Math.toRadians(-110))
                .splineToConstantHeading(specimenSamplePositions[0], 0)
                .splineToConstantHeading(new Vector2d(specimenSamplePositions[1].x, -48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(specimenSamplePositions[2].x, -12), 0)
                .setTangent(Math.PI / 2)
                .lineToY(-48)
                .strafeToLinearHeading(specimenPickupPos, Math.toRadians(-90)) //Cycle specimens
                .strafeToLinearHeading(specimenHangPos, Math.toRadians(90))
                .strafeToLinearHeading(specimenPickupPos, Math.toRadians(-90))
                .strafeToLinearHeading(specimenHangPos, Math.toRadians(90))
                .strafeToLinearHeading(specimenPickupPos, Math.toRadians(-90))
                .strafeToLinearHeading(specimenHangPos, Math.toRadians(90))
                .strafeToLinearHeading(specimenPickupPos, Math.toRadians(-90))
                .strafeToLinearHeading(specimenHangPos, Math.toRadians(90));
*/
        TrajectoryActionBuilder tab2 = myBot.getDrive().actionBuilder(new Pose2d(width / 2, -72 + length / 2, Math.toRadians(90)))
                .lineToY(-34)
                        .strafeToLinearHeading(new Vector2d(26, -38), Math.toRadians(30));

        myBot.runAction(tab1.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1)
                .addEntity(myBot)
                .start();
    }
}