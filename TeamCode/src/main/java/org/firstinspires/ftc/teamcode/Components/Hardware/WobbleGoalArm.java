package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoalArm {

    private Servo wobbleLifter;
    private Servo wobbleGrabber;

    public Mode mode = Mode.REST;
    public GrabberMode grabberMode = GrabberMode.OPEN;

    public enum Mode {
        REST, LIFT;
    }

    public enum GrabberMode {
        OPEN, CLOSED;
    }

    public WobbleGoalArm(HardwareMap hardwareMap, Telemetry telemetry) {

        wobbleLifter = hardwareMap.get(Servo.class, "wobbleLifter");
        wobbleGrabber = hardwareMap.get(Servo.class, "wobbleGrabber");

    }

    public void down() {
        mode = Mode.REST;
        wobbleLifter.setPosition(0.35);
    }


    public void lift() {
        mode = Mode.LIFT;
        wobbleLifter.setPosition(1);
    }

    public void grab() {
        grabberMode = GrabberMode.CLOSED;
        wobbleGrabber.setPosition(0.2);
    }

    public void release() {
        grabberMode = GrabberMode.OPEN;
        wobbleGrabber.setPosition(1);
    }

}
