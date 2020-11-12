package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Hardware.GlobalConfig;

import java.util.concurrent.ExecutorService;

import static org.firstinspires.ftc.teamcode.Components.Hardware.GlobalConfig.*;
import static org.firstinspires.ftc.teamcode.Components.GlobalPositioning.*;

public class OdometryTracker {

    DcMotor rightOdometer, leftOdometer, normalOdometer;
    double rightOdometerPosition, leftOdometerPosition, normalOdometerPosition;

    private ExecutorService odometryUpdaterExecutor;
    private Boolean continueExecution = true;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        //TODO: Map odometers to whichever ports make the most sense based on cabling

        rightOdometer = hardwareMap.get(DcMotor.class, "frontLeft");
        leftOdometer = hardwareMap.get(DcMotor.class, "frontRight");
        normalOdometer = hardwareMap.get(DcMotor.class, "backLeft");

        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        normalOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void updatePosition() {

        double dR = (rightOdometer.getCurrentPosition() - rightOdometerPosition) / countsPerInch;
        double dL = (leftOdometer.getCurrentPosition() - leftOdometerPosition) / countsPerInch;
        double dN = (normalOdometer.getCurrentPosition() - normalOdometerPosition) / countsPerInch;

        double localDx = dN - (normalOffset * (dR - dL) / podDistance);
        double localDy = (dR + dL) / 2;

        robotX += (localDx * Math.cos(robotTheta)) - (localDy * Math.sin(robotTheta));
        robotY += (localDx * Math.sin(robotTheta)) + (localDy * Math.cos(robotTheta));

        robotTheta += (dR - dL) / podDistance;

        rightOdometerPosition += dR * countsPerInch;
        leftOdometerPosition += dL * countsPerInch;
        normalOdometerPosition += dN * countsPerInch;

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
