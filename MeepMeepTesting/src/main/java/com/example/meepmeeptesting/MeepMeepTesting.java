package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

enum Route {
    RED_NEAR,
    RED_FAR,
    BLUE_NEAR,
    BLUE_FAR
}

public class MeepMeepTesting {
    public static final Route ROUTE = Route.RED_NEAR;

    public static final double DELAY = 0.5;
    public static final double MAX_VEL = 60;
    public static final double MAX_ACCEL = 60;
    public static final double MAX_ANGVEL = Math.toRadians(180);
    public static final double MAX_ANGACCEL = Math.toRadians(180);
    public static final double TRACKWIDTH = 15;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;



        switch (ROUTE) {
            case RED_NEAR:
                myBot = redNear(meepMeep);
                break;
//            case RED_FAR:
//                break;
//            case BLUE_NEAR:
//                break;
//            case BLUE_FAR:
//                break;

            default:
                throw new Error("Initializition problem");
        }



//        myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
//                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, -36, Math.toRadians(180)))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(48, 48, 0), Math.PI / 2)
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static RoadRunnerBotEntity redNear(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        Action trajStart = myBot.getDrive().actionBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                .lineToY(34)
//                .waitSeconds(0.4)
//                .turn(Math.toRadians(-90))
//                .lineToX(50)
                .build();
        int propPosition = 2;
        TrajectoryActionBuilder trajPropBuilder = myBot.getDrive().
                actionBuilder(new Pose2d(12, 34, Math.toRadians(270)));
        Action trajProp, reverseTrajProp;
        switch(propPosition){
            case 0: // left
                trajProp = trajPropBuilder.turn(-Math.PI/2).build();
                reverseTrajProp = myBot.getDrive().
                        actionBuilder(new Pose2d(12, 34, Math.toRadians(180))).
                        turn(Math.toRadians(-90)).build();
                break;
            case 1: // center
                trajProp = trajPropBuilder.turn(Math.PI).build();
                reverseTrajProp = myBot.getDrive().
                        actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
                        .turn(Math.toRadians(0)).build();
                break;
            case 2: // right
                trajProp = trajPropBuilder.turn(Math.PI/2).build();
                reverseTrajProp = myBot.getDrive().
                        actionBuilder(new Pose2d(12, 34, 0.0)).
                        turn(-Math.toRadians(-90)).build();
                break;
            default:
                throw new Error("Unknown team prop position");
        }
        Action trajGoToBackdrop = myBot.getDrive().actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
                .splineTo(new Vector2d(25, 47), 0)
                .splineTo(new Vector2d(50, 33), 0)
                .build();
        myBot.runAction(new SequentialAction(
    trajStart,
                trajProp,

                // servo action
                reverseTrajProp,
                trajGoToBackdrop
        ));

        return myBot;
    }

}
