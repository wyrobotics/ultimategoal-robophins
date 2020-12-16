package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Ramp {

    private DcMotor rampMotor;

    private double defaultSpeed = 0.8;

    public Ramp(HardwareMap hardwareMap, Telemetry telemetry) {

        rampMotor = hardwareMap.get(DcMotor.class, "rampMotor");

        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveRamp(double power) { rampMotor.setPower(power); }

    public void moveRamp() { rampMotor.setPower(defaultSpeed); }

}
