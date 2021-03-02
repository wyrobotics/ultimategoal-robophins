package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.Software.HardwareThreadExecutor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Arrays;

public class RRMainRobot extends MainRobot {

    public SampleMecanumDrive drive;

    public RRMainRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        drivebase = null;
        odometryTracker = null;
        hardwareThreadExecutor = null;

        drive = new SampleMecanumDrive(hardwareMap);

        hardwareThreadExecutor = new HardwareThreadExecutor(new ArrayList<Runnable>(Arrays.asList(shooter.shooterRunnable)));

    }

}
