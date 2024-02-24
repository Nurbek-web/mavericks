package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

enum Route {
    RED_NEAR,
    RED_FAR,
    BLUE_NEAR,
    BLUE_FAR
}

public class MeepMeepTesting {
    public static final Route ROUTE = Route.BLUE_NEAR;

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
            case BLUE_NEAR:
                myBot = blueNear(meepMeep);
                break;
            case BLUE_FAR:
                myBot = blueFar(meepMeep);
                break;
            case RED_NEAR:
                myBot = redNear(meepMeep);
                break;
            case RED_FAR:
                myBot = redFar(meepMeep);
                break;

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

    private static RoadRunnerBotEntity blueNear(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder trajBackdrop, trajStart;
        int propPosition = 0;
        Vector2d startingPosition;
        switch(propPosition){
            case 0: // right
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))

                        .strafeToConstantHeading(new Vector2d(12, 35.5))
                        .turn(-Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(8, 35.5))
                        .strafeToConstantHeading(new Vector2d(12, 35.5));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(12, 35.5, 0))
                        .splineToSplineHeading(new Pose2d(37.8, 30, Math.PI), 0);                startingPosition = new Vector2d(37.8, 28.5);
                break;
            case 1: // center
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(12, 30))
                        .strafeToConstantHeading(new Vector2d(12, 34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(20, 33))
                        .splineToSplineHeading(new Pose2d(37.8, 35.5, Math.PI), 0);
                startingPosition = new Vector2d(37.8, 35.5);

                break;
            case 2: // left
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(14, 34), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(18, 34))
                        .strafeToConstantHeading(new Vector2d(14, 34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(14, 34, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(20, 45), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(37.8, 45.5), Math.toRadians(180));
                startingPosition = new Vector2d(37.8, 45.5);

                break;
            default:
                throw new Error("Unknown team prop position");
        }
//
//        TrajectoryActionBuilder fTraj = myBot.getDrive().actionBuilder(new Pose2d(48, 33, Math.PI))
//                        .strafeToConstantHeading(new Vector2d(12, 60))
//                        .strafeToConstantHeading(new Vector2d(-48, 60))
//                .strafeToConstantHeading(new Vector2d(-55, 34));
//        TrajectoryActionBuilder fTrajEnd = myBot.getDrive().actionBuilder(new Pose2d(-55, 34, Math.PI))
//                        .strafeToConstantHeading(new Vector2d(-48, 60))
//                        .strafeToConstantHeading(new Vector2d(12, 60))
//                        .strafeToConstantHeading(new Vector2d(48, 33));

        TrajectoryActionBuilder fTraj = myBot.getDrive().actionBuilder(new Pose2d(startingPosition.x, startingPosition.y, Math.PI))
                .splineToConstantHeading(new Vector2d(12, 60), Math.PI)
                .strafeToConstantHeading(new Vector2d(-48, 60))
                .splineToConstantHeading(new Vector2d(-55, 34), Math.PI/2);
        TrajectoryActionBuilder fTrajEnd = myBot.getDrive().actionBuilder(new Pose2d(-55, 34, Math.PI))
                .strafeToConstantHeading(new Vector2d(-55, 60))
                .strafeToConstantHeading(new Vector2d(12, 60))
                .splineToConstantHeading(new Vector2d(48, 33), 0);

        myBot.runAction(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build()
//                fTraj.build(),
//                fTrajEnd.build()
        ));

        return myBot;
    }

    private static RoadRunnerBotEntity blueFar(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        TrajectoryActionBuilder trajStart, trajBackdrop;
            int propPosition = 1;

        switch(propPosition){
            case 0: // right
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(-35, 60, Math.toRadians(90)))

                        .strafeToConstantHeading(new Vector2d(-35, 37))
                        .turn(-Math.toRadians(90));
//                        .strafeToConstantHeading(new Vector2d(5, 37))
//                        .strafeToConstantHeading(new Vector2d(12, 37));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-36, 37, 0))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .strafeToConstantHeading(new Vector2d(20, 58))
                        .splineToSplineHeading(new Pose2d(40.7, 31, Math.PI), 0);
                break;
            case 1: // center
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(-35, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-35, 30))
                        .strafeToConstantHeading(new Vector2d(-35, 36));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-35, 36, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .strafeToConstantHeading(new Vector2d(20, 58))
                        .splineToSplineHeading(new Pose2d(39.5, 38, Math.PI), 0);

                break;
            case 2: // left
                trajStart = myBot.getDrive()
                        .actionBuilder(new Pose2d(-35, 58, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-36, 34))
                        .turn(Math.toRadians(90));
//                        .strafeToConstantHeading(new Vector2d(18, 34))
//                        .strafeToConstantHeading(new Vector2d(14, 34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-35, 34, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .strafeToConstantHeading(new Vector2d(20, 58))
                        .strafeToLinearHeading(new Vector2d(40.7, 45.5), Math.toRadians(180));
                break;
            default:
                throw new Error("Unknown team prop position");
        }


        myBot.runAction(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build()
        ));

        return myBot;
    }

    private static RoadRunnerBotEntity redNear(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        DriveShim drive = myBot.getDrive();

        TrajectoryActionBuilder trajBackdrop, trajStart;
        int propPosition = 0;
        Vector2d startingPosition;
        switch(propPosition){
            case 0: // right
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(14, -34), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(18, -34))
                        .strafeToConstantHeading(new Vector2d(14, -34));
                trajBackdrop = drive
                        .actionBuilder(new Pose2d(14, -34, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(14, -50))
                        .strafeToConstantHeading(new Vector2d(35, -50))
                        .strafeToConstantHeading(new Vector2d(40.8, -41.5));
                break;
            case 1: // center
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(12, -30))
                        .strafeToConstantHeading(new Vector2d(12, -34));
                trajBackdrop = drive.actionBuilder(new Pose2d(12, -34, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(40.8, -34.5, Math.PI), 0);
                break;
            case 2: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(12, -36), 0)
                        .strafeToConstantHeading(new Vector2d(8, -36))
                        .strafeToConstantHeading(new Vector2d(10.5, -36));

                trajBackdrop = drive.actionBuilder(new Pose2d(10.5, -36, 0))
                        .splineToSplineHeading(new Pose2d(38.8, -29, Math.PI), 0);
                break;
            default:
                throw new Error("Unknown team prop position");
        }
//
//        TrajectoryActionBuilder fTraj = myBot.getDrive().actionBuilder(new Pose2d(startingPosition.x, startingPosition.y, Math.PI))
//                .splineToConstantHeading(new Vector2d(12, -60), Math.PI)
//                .strafeToConstantHeading(new Vector2d(-48, -60))
//                .splineToConstantHeading(new Vector2d(-55, -34), Math.PI/2);
//        TrajectoryActionBuilder fTrajEnd = myBot.getDrive().actionBuilder(new Pose2d(-55, -34, Math.PI))
//                .strafeToConstantHeading(new Vector2d(-55, -60))
//                .strafeToConstantHeading(new Vector2d(12, -60))
//                .splineToConstantHeading(new Vector2d(37.8, -33), 0);

        myBot.runAction(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build()
//                fTraj.build(),
//                fTrajEnd.build()
        ));

        return myBot;

    }

    private static RoadRunnerBotEntity redFar(MeepMeep meepMeep) {
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        TrajectoryActionBuilder trajStart, trajBackdrop;
        int propPosition = 0;
        Vector2d startingPosition;
        switch(propPosition){
            case 0: // right
                trajStart = myBot.getDrive().actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-36, -34), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(-32, -34))
                        .strafeToConstantHeading(new Vector2d(-36, -34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-36, -34, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-34, -60))
                        .strafeToConstantHeading(new Vector2d(10, -60))
                        .splineToSplineHeading(new Pose2d(37.8, -45.5, Math.PI), 0);
                startingPosition = new Vector2d(37.8, -45.5);
                break;
            case 1: // center
                trajStart = myBot.getDrive().actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(-34, -30))
                        .strafeToConstantHeading(new Vector2d(-34, -34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-34, -34, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(-34, -60))
                        .strafeToConstantHeading(new Vector2d(10, -60))
                        .splineToSplineHeading(new Pose2d(37.8, -33, Math.PI), 0);
                startingPosition = new Vector2d(37.8, -33);
                break;
            case 2: // left
                trajStart = myBot.getDrive().actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0))
                        .strafeToConstantHeading(new Vector2d(-38, -34))
                        .strafeToConstantHeading(new Vector2d(-34, -34));
                trajBackdrop = myBot.getDrive().actionBuilder(new Pose2d(-34, -34, Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(-34, -58))
                        .strafeToConstantHeading(new Vector2d(12, -58))
                        .splineToSplineHeading(new Pose2d(37.8, -45, Math.PI), 0);
                startingPosition = new Vector2d(37.8, -45);
                break;
            default:
                throw new Error("Unknown team prop position");
        }

        TrajectoryActionBuilder fTraj = myBot.getDrive().actionBuilder(new Pose2d(startingPosition.x, startingPosition.y, Math.PI))
                .splineToConstantHeading(new Vector2d(12, -60), Math.PI)
                .strafeToConstantHeading(new Vector2d(-48, -60))
                .splineToConstantHeading(new Vector2d(-55, -34), Math.PI/2);
        TrajectoryActionBuilder fTrajEnd = myBot.getDrive().actionBuilder(new Pose2d(-55, -34, Math.PI))
                .strafeToConstantHeading(new Vector2d(-55, -60))
                .strafeToConstantHeading(new Vector2d(12, -60))
                .splineToConstantHeading(new Vector2d(37.8, -33), 0);

        myBot.runAction(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build(),
                fTraj.build(),
                fTrajEnd.build()
        ));

        return myBot;
    }

}
