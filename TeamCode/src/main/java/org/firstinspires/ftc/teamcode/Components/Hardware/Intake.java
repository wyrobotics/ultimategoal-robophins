package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotor intake;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        intake = hardwareMap.get(DcMotor.class, "intake");

    }

}
