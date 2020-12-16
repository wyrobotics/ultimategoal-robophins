package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.MainRobot;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class Turn extends LinearOpMode {

    private MainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    double contourArea = 0;

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() throws InterruptedException {

        mainRobot = new MainRobot(hardwareMap, telemetry);

        mainRobot.wobbleGoalArm.grab();

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();


        mainRobot.drivebase.timedMovement(0, 0, 0.5, 2275, 600);

        mainRobot.hardwareThreadExecutor.shutdownExecutor();

        mainRobot.drivebase.discOrtho(0,0,0);

    }

}
