package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private DcMotor leftShooter;
    private DcMotor rightShooter;

    //private DigitalChannel shooterSwitch;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //shooterSwitch = hardwareMap.get(DigitalChannel.class, "shooterSwitch");

    }

    public void simpleShoot() {
        leftShooter.setPower(0.8);
        rightShooter.setPower(0.8);
    }

    public void simpleShoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

}
