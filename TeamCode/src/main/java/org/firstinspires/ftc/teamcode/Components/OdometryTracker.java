package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryTracker {

    DcMotor rightOdometer, leftOdometer, normalOdometer;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        //TODO: Map odometers to whichever ports make the most sense based on cabling

        rightOdometer = hardwareMap.get(DcMotor.class, "frontLeft");
        leftOdometer = hardwareMap.get(DcMotor.class, "frontRight");
        normalOdometer = hardwareMap.get(DcMotor.class, "backLeft");

    }



}
