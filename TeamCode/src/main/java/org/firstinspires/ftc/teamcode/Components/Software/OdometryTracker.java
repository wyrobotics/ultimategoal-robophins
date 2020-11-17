package org.firstinspires.ftc.teamcode.Components.Software;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.ExecutorService;

import static org.firstinspires.ftc.teamcode.Components.Hardware.GlobalConfig.*;
import static org.firstinspires.ftc.teamcode.Components.Software.GlobalPositioning.*;

public class OdometryTracker {

    public DcMotor rightOdometer, leftOdometer, normalOdometer;
    //TODO: NOT PUBLICCC
    public double rightOdometerPosition, leftOdometerPosition, normalOdometerPosition;

    private ExecutorService odometryUpdaterExecutor;
    private Boolean continueExecution = true;

    Telemetry telemetry;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        rightOdometer = hardwareMap.get(DcMotor.class, "rightOdometer");
        leftOdometer = hardwareMap.get(DcMotor.class, "leftShooter");
        normalOdometer = hardwareMap.get(DcMotor.class, "intakeMotor");

        this.telemetry = telemetry;

        //rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //normalOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void updatePosition() {

        double newRight = rightOdometer.getCurrentPosition();
        double newLeft = leftOdometer.getCurrentPosition();
        double newNorm = normalOdometer.getCurrentPosition();

        double dR = -(newRight - rightOdometerPosition) / countsPerInch;
        double dL = (newLeft - leftOdometerPosition) / countsPerInch;
        double dN = (newNorm - normalOdometerPosition) / countsPerInch;

        double localDx = dN - (normalOffset * (dR - dL) / podDistance);
        double localDy = (dR + dL) / 2;

        robotX += (localDx * Math.cos(robotTheta)) - (localDy * Math.sin(robotTheta));
        robotY += (localDx * Math.sin(robotTheta)) + (localDy * Math.cos(robotTheta));

        robotTheta += (dL - dR) / podDistance;

        rightOdometerPosition = newRight;
        leftOdometerPosition = newLeft;
        normalOdometerPosition = newNorm;


    }

    private Runnable odometryUpdaterRunnable = new Runnable() {
        @Override
        public void run() {
            continueExecution = true;

            rightOdometerPosition = rightOdometer.getCurrentPosition();
            leftOdometerPosition = leftOdometer.getCurrentPosition();
            normalOdometerPosition = normalOdometer.getCurrentPosition();

            while (continueExecution && !Thread.currentThread().isInterrupted()) {
                updatePosition();
                sleep(10);
            }
        }
    };

    private void sleep(double milliseconds) {

        double time = System.currentTimeMillis();

        while (System.currentTimeMillis() - time < milliseconds) { }

    }

    public void startOdometry() {
        odometryUpdaterExecutor = ThreadPool.newSingleThreadExecutor("Odometry Updater");
        odometryUpdaterExecutor.execute(odometryUpdaterRunnable);
    }

    public void shutdownOdometry() {
        continueExecution = false;
        odometryUpdaterExecutor.shutdownNow();
        odometryUpdaterExecutor = null;
    }

}
