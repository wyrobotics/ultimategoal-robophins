package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivebase {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public Drivebase(HardwareMap hardwareMap, Telemetry telemetry) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void discOrtho(double leftStickX, double leftStickY, double turningPower) {

        double theta = Math.atan2(leftStickY, leftStickX);
        double magnitude = Math.hypot(leftStickX, leftStickY);

        double flbr = magnitude * Math.cos(theta - (Math.PI / 4));
        double frbl = magnitude * Math.sin(theta - (Math.PI / 4));

        //double rotChange = Math.min(Math.abs(turningPower),
         //       Math.min(1 - Math.abs(flbr), 1 - Math.abs(frbl)));

        frontLeft.setPower(flbr + turningPower);
        frontRight.setPower(frbl - turningPower);
        backLeft.setPower(frbl + turningPower);
        backRight.setPower(flbr - turningPower);

    }

}
