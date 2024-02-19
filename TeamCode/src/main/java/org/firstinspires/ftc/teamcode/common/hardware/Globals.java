package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.opmode.testing.device.Hang;

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
    public static boolean INTAKE_LOWERED = true;


    public enum LiftLevel {
        FIRST,
        SECOND,
        THIRD
    }

    public enum HangServoState {
        OPENED,
        CLOSED,
        REVERSE
    }

    public static LiftLevel liftLevel = LiftLevel.FIRST;

    public static HangServoState HangServoOpened = HangServoState.CLOSED;

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


    public static boolean outtakeClosed = false;
    public static void closeOuttake(){ outtakeClosed = true; }
    public static void changeLiftLevel(LiftLevel lev){
        liftLevel = lev;
    }
}
