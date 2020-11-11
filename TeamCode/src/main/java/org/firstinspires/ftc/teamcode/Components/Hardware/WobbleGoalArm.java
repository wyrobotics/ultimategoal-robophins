package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm {

    private Servo wobbleLifter;
    private Servo wobbleGrabber;

    public WobbleGoalArm(HardwareMap hardwareMap, Telemetry telemetry) {

        wobbleLifter = hardwareMap.get(Servo.class, "wobbleLifter");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");

    }

}
