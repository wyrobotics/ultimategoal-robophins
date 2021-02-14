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

    Telemetry telemetry;

    private Boolean continueExecution = true; //thread
    private volatile Boolean runController = false; //pid

    private DcMotor leftShooter;
    private DcMotor rightShooter;

    private Servo flicker;
    public boolean flicked = false;

    //private DigitalChannel shooterSwitch;

    //1440 ppr
    //setpoint in ticks/sec
    public volatile double setpoint = 0;
    private double[] lastPos = new double[3];

    private double shooterPower = 0;

    private double kP = 0.0003, kI = 0, kD = 0;

    public double lastTime;
    public double lastCount;

    public double teleLastTime = 0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        flicker = hardwareMap.get(Servo.class, "flicker");

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        //Right shooter has encoder

        //shooterSwitch = hardwareMap.get(DigitalChannel.class, "shooterSwitch");

    }

    public void flick() {
        flicker.setPosition(0.8);
        flicked = true;
    }

    public void unflick() {
        flicker.setPosition(0.4);
        flicked = false;
    }

    public void simpleShoot() {
        leftShooter.setPower(0.8);
        rightShooter.setPower(0.8);
        shooterPower = 0.8;
    }

    public void simpleShoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
        shooterPower = power;
    }

    public void enableController() { runController = true; }
    public void disableController() { runController = false; }

    public double getRPM() {
        double newPos = rightShooter.getCurrentPosition();
        double newTime = System.currentTimeMillis();
        double result = (newPos - lastCount) / (newTime - lastTime);

        lastCount = newPos;
        lastTime = newTime;

        return result;
    }

    public void setSetpoint(double setpoint) { this.setpoint = setpoint; }

    public double getShooterPower() { return leftShooter.getPower(); }
    public double getShooterPower(int i) {
        switch(i) {
            case -1: return rightShooter.getPower();
            case 0: return (leftShooter.getPower() + rightShooter.getPower()) / 2;
            default: return leftShooter.getPower();
        }
    }

    public void initPos() {

        lastPos[0] = rightShooter.getCurrentPosition();
        sleep(10);
        lastPos[1] = rightShooter.getCurrentPosition();
        sleep(10);
        lastPos[2] = rightShooter.getCurrentPosition();

    }

    public void shooterController(double integrator) {

        lastPos[0] = lastPos[1];
        lastPos[1] = lastPos[2];
        lastPos[2] = rightShooter.getCurrentPosition();

        double newTime = System.currentTimeMillis();

        double dt = (newTime - lastTime) / 1000;

        double e = setpoint - ((lastPos[2] - lastPos[1]) / dt);

        integrator += e * dt;

        double u = (kP * e) + (kI * integrator) + (kD * (e - (setpoint - ((lastPos[1] - lastPos[0]) / dt))) / dt);

        shooterPower = Math.max(-1, Math.min(1, shooterPower + u));

        telemetry.addData("e: ", e);
        telemetry.addData("u: ", u);
        telemetry.addData("Shooter power: ", shooterPower);
        telemetry.addData("ticks per sec: ", ((lastPos[2] - lastPos[1]) / dt));
        telemetry.addData("lastPos[1]: ", lastPos[1]);
        telemetry.addData("lastPos[2]: ", lastPos[2]);
        telemetry.addData("dt: ", dt);

        rightShooter.setPower(shooterPower);
        leftShooter.setPower(shooterPower);

        lastTime = newTime;

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
                sleep(200);
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
