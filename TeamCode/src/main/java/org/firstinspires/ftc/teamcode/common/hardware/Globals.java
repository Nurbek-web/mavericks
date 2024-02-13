package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.vision.Location;

public class Globals {

    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static Location ALLIANCE = Location.RED;
    public static boolean IS_AUTO = false;

    /**
     * Robot State Constants
     */
    public enum IntakeState {
        INTAKING,
        NOT_WORKING,
        REVERSE
    }
    public static IntakeState IS_INTAKING = IntakeState.NOT_WORKING;
    public static boolean INTAKE_LOWERED = false;

    public enum LiftLevel {
        FIRST,
        SECOND,
        THIRD
    }

    public static LiftLevel liftRaised = LiftLevel.FIRST;

    public static void startIntaking() {
        IS_INTAKING = IntakeState.INTAKING;
    }

    public static void stopIntaking(){
        IS_INTAKING = IntakeState.NOT_WORKING;
    }

    public static void releaseIntaking(){
        IS_INTAKING = IntakeState.REVERSE;
    }

    public static void lowerIntake() {
        INTAKE_LOWERED = true;
    }

    public static void raiseIntake() {
        INTAKE_LOWERED = false;
    }
}
