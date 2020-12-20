package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Hardware.Drivebase;
import org.firstinspires.ftc.teamcode.Components.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Components.Hardware.Ramp;
import org.firstinspires.ftc.teamcode.Components.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Components.Hardware.WobbleGoalArm;
import org.firstinspires.ftc.teamcode.Components.Software.HardwareThreadExecutor;
import org.firstinspires.ftc.teamcode.Components.Software.OdometryTracker;

import java.util.ArrayList;
import java.util.Arrays;

public class  MainRobot {

    public Shooter shooter;
    public Ramp ramp;
    public Intake intake;
    public WobbleGoalArm wobbleGoalArm;
    public Drivebase drivebase;

    public OdometryTracker odometryTracker;
    public HardwareThreadExecutor hardwareThreadExecutor;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        wobbleGoalArm = new WobbleGoalArm(hardwareMap, telemetry);
        drivebase = new Drivebase(hardwareMap, telemetry);
        ramp = new Ramp(hardwareMap, telemetry);

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);

        hardwareThreadExecutor = new HardwareThreadExecutor(
                new ArrayList<Runnable>(Arrays.asList(odometryTracker.odometryUpdaterRunnable, shooter.shooterRunnable)));

    }

    public void deng(double millis) {
        double init = System.currentTimeMillis();
        while(System.currentTimeMillis() < init + millis) { }
    }
    public void jig(int n) {
        for(int i = 0; i < n; i++) {
            drivebase.timedMovement(0, .5, 0, 150, 50);
            deng(50);
            drivebase.timedMovement(0, -.5, 0, 150, 50);
            deng(50);
        }
    }
    public void closeAllThreads() {
        odometryTracker.shutdownOdometry();
        shooter.shutdownShooter();
        hardwareThreadExecutor.shutdownExecutor();
    }

}
