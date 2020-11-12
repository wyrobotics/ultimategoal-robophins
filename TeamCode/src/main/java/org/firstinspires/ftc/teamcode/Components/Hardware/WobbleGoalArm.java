package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm {

    private Servo wobbleLifter;
    private Servo wobbleGrabber;

    private Mode mode = Mode.REST;

    public enum Mode {
        REST, GRAB, LIFT;
    }

    public WobbleGoalArm(HardwareMap hardwareMap, Telemetry telemetry) {

        wobbleLifter = hardwareMap.get(Servo.class, "wobbleLifter");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");

    }

    public void down() {
        mode = Mode.REST;
        wobbleLifter.setPosition(0);
    }

    public void grabPosition() {
        mode = Mode.GRAB;
        wobbleLifter.setPosition(0.5);
    }

    public void lift() {
        mode = Mode.LIFT;
        wobbleGrabber.setPosition(0.1);
        wobbleLifter.setPosition(1);
    }


    public void grab() { wobbleGrabber.setPosition(0.1); }

    public void release() { if(mode != Mode.LIFT) { wobbleGrabber.setPosition(0.7); }; }

}
