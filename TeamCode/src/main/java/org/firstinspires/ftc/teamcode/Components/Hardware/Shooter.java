package org.firstinspires.ftc.teamcode.Components.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;

public class Shooter {

    private Boolean continueExecution = true;
    private Boolean runController = false;

    private DcMotor leftShooter;
    private DcMotor rightShooter;

    private Servo flicker;
    public boolean flicked = false;

    //private DigitalChannel shooterSwitch;

    public double setpoint = 0;
    private double[] lastPos = new double[3];

    private double kP = 0, kI = 0, kD = 0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        flicker = hardwareMap.get(Servo.class, "flicker");

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right shooter has encoder

        //shooterSwitch = hardwareMap.get(DigitalChannel.class, "shooterSwitch");

    }

    public void flick() {
        flicker.setPosition(0.8);
        flicked = true;
    }

    public void unflick() {
        flicker.setPosition(0.2);
        flicked = false;
    }

    public void simpleShoot() {
        leftShooter.setPower(0.8);
        rightShooter.setPower(0.8);
    }

    public void simpleShoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void enableController() { runController = true; }
    public void disableController() { runController = false; }

    public void setSetpoint(double setpoint) { this.setpoint = setpoint; }

    public double getShooterPower() { return leftShooter.getPower(); }
    public double getShooterPower(int i) {
        switch(i) {
            case -1: return rightShooter.getPower();
            case 0: return (leftShooter.getPower() + rightShooter.getPower()) / 2;
            default: return leftShooter.getPower();
        }
    }

    public void shooterController(double integrator) {

        lastPos[0] = lastPos[1];
        lastPos[1] = lastPos[2];
        lastPos[2] = rightShooter.getCurrentPosition();

        double e = setpoint - ((lastPos[2] - lastPos[1]) / 0.01);

        integrator += e * 0.01;

        double u = (kP * e) + (kI * integrator) + (kD * (e - (setpoint - ((lastPos[1] - lastPos[10]) / 0.01))) / 0.01);

        rightShooter.setPower(u);
        leftShooter.setPower(u);

    }

    public Runnable shooterRunnable = new Runnable() {
        @Override
        public void run() {
            continueExecution = true;
            lastPos[1] = rightShooter.getCurrentPosition();
            lastPos[2] = rightShooter.getCurrentPosition();
            double integrator = 0;
            while(continueExecution && !Thread.currentThread().isInterrupted()) {
                if(runController) { shooterController(integrator); }
                sleep(10);
            }
        }
    };

    public void shutdownShooter() {
        continueExecution = false;
    }

    private void sleep(double milliseconds) {

        double time = System.currentTimeMillis();

        while (System.currentTimeMillis() - time < milliseconds) { }

    }

}
