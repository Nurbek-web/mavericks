package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class SampleMecanumDrivetrain {
    private final RobotHardware robot = RobotHardware.getInstance();

    public SampleMecanumDrivetrain() {
    }

    // Use the IMU to return the angle of the robot.
    public double getAngle() {
        return 0;
    }

    // Remaps the given angle into the range (-180, 180].
//    public static double normalize(double degrees) {
//        double normalized_angle = Angle.normalizePositive(degrees);
//        if (normalized_angle > 180) {
//            normalized_angle -= 360;
//        }
//        return normalized_angle;
//    }

//    public void driveRobotCentric(double drive, double angle, double strafe) {
//        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
//        double fRightPow, bRightPow, fLeftPow, bLeftPow;
//
//        // Compute how much you need to turn to maintain that angle
//        // so the robot turns smoothly
//
//
//        double rotX = drive- strafe;
//        double rotY = drive + strafe;
//
//        // Do the math found in GM0
//        double denominator = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(turn), 1);
//        fLeftPow = (rotY + rotX + turn) / denominator;
//        bLeftPow = (rotY - rotX + turn) / denominator;
//        fRightPow = (rotY - rotX - turn) / denominator;
//        bRightPow = (rotY + rotX - turn) / denominator;
//
//        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
//    }


    // Drive or Strafe to at some power while turning to some angle.
    public void driveRobotCentric(double y, double x, double rx) {
        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
        double fRightPow, bRightPow, fLeftPow, bLeftPow;

        // Do the math found in GM0
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        fLeftPow = (y + x + rx) / denominator;
        bLeftPow = (y - x + rx) / denominator;
        fRightPow = (y - x - rx) / denominator;
        bRightPow = (y + x - rx) / denominator;

        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    }

    public void driveFieldCentric(double y, double x, double rx, double botHeading) {
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        setDrivePowers(backLeftPower, frontLeftPower, backRightPower, frontRightPower);
    }

    public void setDrivePowers(double bLeftPow, double fLeftPow, double bRightPow, double fRightPow) {
        robot.dtBackLeftMotor.setPower(bLeftPow);
        robot.dtFrontLeftMotor.setPower(fLeftPow);
        robot.dtBackRightMotor.setPower(bRightPow);
        robot.dtFrontRightMotor.setPower(fRightPow);
    }

    public void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }

    // Misc. Functions / Overloaded Method Storage
//    private double getVoltage() {
//        double voltage = Double.MIN_VALUE;
//        for (LynxModule hub : allHubs) {
//            voltage = Math.max(voltage, hub.getInputVoltage(VoltageUnit.VOLTS));
//        }
//
//        return voltage;
//    }

}