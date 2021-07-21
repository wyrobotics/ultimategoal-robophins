package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wacker {

    private Servo wacker;

    public Wacker(HardwareMap hardwareMap, Telemetry telemetry) {

        wacker = hardwareMap.get(Servo.class, "wacker");

    }

    public void lowerWacker() {
        wacker.setPosition(1);
    }

    public void upperWacker(){
        wacker.setPosition(0.90);
    }

}
