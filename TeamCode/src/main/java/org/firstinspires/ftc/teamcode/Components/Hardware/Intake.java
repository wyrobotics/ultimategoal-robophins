package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

    }

    public void intake() { intakeMotor.setPower(1); }

    public void intake(double power) { intakeMotor.setPower(power); }

}
