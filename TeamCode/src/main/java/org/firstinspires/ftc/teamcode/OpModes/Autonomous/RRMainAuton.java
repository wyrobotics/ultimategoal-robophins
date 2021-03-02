package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RRMainRobot;
import org.firstinspires.ftc.teamcode.Components.Software.Pipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class RRMainAuton extends LinearOpMode {

    RRMainRobot mainRobot;

    OpenCvInternalCamera phoneCam;

    Pipeline pipeline = new Pipeline();

    int stackHeight(double area) { return (area < 100) ? 0 : ((area > 450) ? 2 : 1); }

    @Override
    public void runOpMode() {

        mainRobot = new RRMainRobot(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(pipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        //phoneCam.setFlashlightEnabled(true);

        mainRobot.wobbleGoalArm.grab();

        mainRobot.shooter.setpoint = 0;

        mainRobot.shooter.initPos();

        mainRobot.shooter.enableController();

        waitForStart();

        mainRobot.hardwareThreadExecutor.initiateExecutor();
        //mainRobot.shooter.startShooter();

        telemetry.addData("Stack Height:", stackHeight(pipeline.getContourArea()));
        double height = stackHeight(pipeline.getContourArea());
        telemetry.update();

        phoneCam.stopStreaming();



    }

}
