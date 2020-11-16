package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivebase {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double maxPower = 1;

    public Drivebase(HardwareMap hardwareMap, Telemetry telemetry) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        frontLeft.setPower(maxPower * (flbr - turningPower));
        frontRight.setPower(maxPower * (frbl + turningPower));
        backLeft.setPower(maxPower * (frbl - turningPower));
        backRight.setPower(maxPower * (flbr + turningPower));

    }

}
